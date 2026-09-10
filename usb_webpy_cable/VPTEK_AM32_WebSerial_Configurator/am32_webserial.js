'use strict';

/*
 * VPTEK AM32 Web Serial backend
 *
 * Browser -> Web Serial -> ESP32-C3 USB CDC raw bridge -> GPIO21 -> AM32 bootloader
 *
 * The ESP bridge runs HOST_ECHO=true. Every transmitted frame is therefore
 * echoed by the ESP before the real ESC response. This backend consumes and
 * verifies that echo exactly like the direct ConfigTool path.
 */

const AM32 = Object.freeze({
  BAUD: 19200,
  ACK_OK: 0x30,
  ACK_BAD_CMD: 0xC1,
  ACK_BAD_CRC: 0xC2,
  CMD_RUN: 0x00,
  CMD_PROG_FLASH: 0x01,
  CMD_READ_FLASH: 0x03,
  CMD_SET_BUFFER: 0xFE,
  CMD_SET_ADDRESS: 0xFF,
  APP_START: 0x1000,
  BLOCK_SIZE: 128,
  EEPROM_BYTES: 48,
  BOOT_INIT: Uint8Array.from([
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,
    0x0D,0x42,0x4C,0x48,0x65,0x6C,0x69,0xF4,0x7D
  ]),
  PROGRAM_FLASH_FRAME: Uint8Array.from([0x01,0x01,0xC0,0x50])
});

const EEPROM_FIELDS = Object.freeze({
  max_ramp:5,
  minimum_duty_cycle:6,
  disable_stick_calibration:7,
  absolute_voltage_cutoff:8,
  current_P:9,
  current_I:10,
  current_D:11,
  active_brake_power:12,
  dir_reversed:17,
  bi_direction:18,
  use_sine_start:19,
  comp_pwm:20,
  variable_pwm:21,
  stuck_rotor_protection:22,
  advance_level:23,
  pwm_frequency:24,
  startup_power:25,
  motor_kv:26,
  motor_poles:27,
  brake_on_stop:28,
  stall_protection:29,
  beep_volume:30,
  telemetry_on_interval:31,
  servo_low_threshold:32,
  servo_high_threshold:33,
  servo_neutral:34,
  servo_dead_band:35,
  low_voltage_cut_off:36,
  low_cell_volt_cutoff:37,
  rc_car_reverse:38,
  use_hall_sensors:39,
  sine_changeover:40,
  drag_brake_strength:41,
  driving_brake_strength:42,
  temperature_limit:43,
  current_limit:44,
  sine_mode_power:45,
  input_type:46,
  auto_advance:47
});

function delay(ms){
  return new Promise(resolve=>setTimeout(resolve,ms));
}

function hexByte(value){
  return '0x'+Number(value).toString(16).toUpperCase().padStart(2,'0');
}

function bytesEqual(a,b){
  if(!a||!b||a.length!==b.length)return false;
  for(let i=0;i<a.length;i++)if(a[i]!==b[i])return false;
  return true;
}

function crc16(data){
  let crc=0;
  for(const input of data){
    crc^=input;
    for(let bit=0;bit<8;bit++){
      crc=(crc&1)?((crc>>>1)^0xA001):(crc>>>1);
      crc&=0xFFFF;
    }
  }
  return crc;
}

function withCrc(data){
  const out=new Uint8Array(data.length+2);
  out.set(data,0);
  const crc=crc16(data);
  out[data.length]=crc&0xFF;
  out[data.length+1]=(crc>>>8)&0xFF;
  return out;
}

function checkDataCrc(frame,dataLength){
  const received=frame[dataLength]|(frame[dataLength+1]<<8);
  return received===crc16(frame.slice(0,dataLength));
}

class SerialTransport {
  constructor(){
    this.port=null;
    this.reader=null;
    this.writer=null;
    this.rx=[];
    this.readLoopPromise=null;
    this.readLoopError=null;
    this.closing=false;
  }

  get isOpen(){
    return !!(this.port&&this.writer&&this.reader);
  }

  async open(){
    if(!('serial' in navigator)){
      throw new Error('Web Serial is not supported. Use desktop Chrome or Edge and open this page from localhost/HTTPS.');
    }

    if(this.isOpen)return;

    if(!this.port){
      // Must be reached directly from a user click (Connect Device).
      this.port=await navigator.serial.requestPort();
    }

    try{
      await this.port.open({
        baudRate:AM32.BAUD,
        dataBits:8,
        stopBits:1,
        parity:'none',
        flowControl:'none',
        bufferSize:4096
      });
    }catch(error){
      // A previously selected port can become stale after unplug/replug.
      this.port=null;
      throw new Error('Cannot open USB serial port: '+(error&&error.message?error.message:error));
    }

    this.writer=this.port.writable.getWriter();
    this.reader=this.port.readable.getReader();
    this.rx.length=0;
    this.readLoopError=null;
    this.closing=false;
    this.readLoopPromise=this._readLoop();

    // Give USB CDC a moment to settle without resetting the ESP bridge protocol.
    await delay(40);
    this.rx.length=0;
  }

  async _readLoop(){
    try{
      while(!this.closing){
        const {value,done}=await this.reader.read();
        if(done)break;
        if(value){
          for(const byte of value)this.rx.push(byte);
        }
      }
    }catch(error){
      if(!this.closing)this.readLoopError=error;
    }
  }

  clearRx(){
    this.rx.length=0;
  }

  async write(data){
    if(!this.isOpen)throw new Error('USB serial port is not open');
    if(!(data instanceof Uint8Array))data=Uint8Array.from(data);
    await this.writer.write(data);
  }

  async readExact(count,timeoutMs=1200){
    const deadline=performance.now()+timeoutMs;
    while(this.rx.length<count){
      if(this.readLoopError){
        throw new Error('USB serial read failed: '+this.readLoopError.message);
      }
      if(performance.now()>=deadline){
        throw new Error(`Serial timeout: expected ${count} byte(s), received ${this.rx.length}`);
      }
      await delay(1);
    }
    return Uint8Array.from(this.rx.splice(0,count));
  }

  async readOptional(timeoutMs=12){
    const deadline=performance.now()+timeoutMs;
    while(this.rx.length===0&&performance.now()<deadline){
      if(this.readLoopError)throw this.readLoopError;
      await delay(1);
    }
    if(this.rx.length===0)return null;
    return this.rx.shift();
  }

  unshift(byte){
    if(byte!==null&&byte!==undefined)this.rx.unshift(byte&0xFF);
  }

  async sendWithEcho(data,timeoutMs=1800){
    if(this.rx.length!==0){
      throw new Error(`Unexpected ${this.rx.length} stale serial byte(s) before TX`);
    }
    await this.write(data);
    const echo=await this.readExact(data.length,timeoutMs);
    if(!bytesEqual(echo,data)){
      let mismatch=-1;
      for(let i=0;i<data.length;i++){
        if(echo[i]!==data[i]){ mismatch=i; break; }
      }
      throw new Error(`USB/ESC echo mismatch at byte ${mismatch}`);
    }
  }

  async close(){
    this.closing=true;
    try{ if(this.reader)await this.reader.cancel(); }catch(_e){}
    try{ if(this.reader)this.reader.releaseLock(); }catch(_e){}
    try{ if(this.writer)this.writer.releaseLock(); }catch(_e){}
    this.reader=null;
    this.writer=null;
    try{ if(this.port)await this.port.close(); }catch(_e){}
    this.rx.length=0;
  }
}

class AM32DirectProtocol {
  constructor(transport){
    this.t=transport;
    this.deviceInfo=null;
    this.flashCode=0;
    this.eepromPhysical=0;
  }

  static eepromPhysicalFromFlashCode(code){
    switch(code){
      case 0x1F:return 0x07C00;
      case 0x35:return 0x0F800;
      case 0x2B:return 0x1F800;
      default:return 0;
    }
  }

  protocolAddressFromPhysical(physical){
    if(this.flashCode===0x2B)physical>>>=2;
    return physical&0xFFFF;
  }

  async connect(){
    this.t.clearRx();
    await this.t.sendWithEcho(AM32.BOOT_INIT,1500);
    const info=await this.t.readExact(9,1200);

    if(info[8]!==AM32.ACK_OK){
      throw new Error('Invalid DeviceInfo ACK: '+hexByte(info[8]));
    }
    if(info[0]!==0x34||info[1]!==0x37||info[2]!==0x31){
      throw new Error('Unexpected MCU DeviceInfo; expected AT32F421 family 471');
    }

    const eepromPhysical=AM32DirectProtocol.eepromPhysicalFromFlashCode(info[4]);
    if(!eepromPhysical){
      throw new Error('Unsupported AM32 flash code '+hexByte(info[4]));
    }

    this.deviceInfo=info;
    this.flashCode=info[4];
    this.eepromPhysical=eepromPhysical;
    return info;
  }

  async readAck(timeoutMs=1000,label='command'){
    const ack=(await this.t.readExact(1,timeoutMs))[0];
    if(ack!==AM32.ACK_OK){
      const name=ack===AM32.ACK_BAD_CRC?'BAD CRC':ack===AM32.ACK_BAD_CMD?'BAD CMD':'BAD ACK';
      throw new Error(`${label}: ${name} (${hexByte(ack)})`);
    }
    return ack;
  }

  async setAddressPhysical(physicalAddress){
    const address=this.protocolAddressFromPhysical(physicalAddress);
    const command=withCrc(Uint8Array.from([
      AM32.CMD_SET_ADDRESS,
      0x00,
      (address>>>8)&0xFF,
      address&0xFF
    ]));
    await this.t.sendWithEcho(command);
    await this.readAck(1000,'SET_ADDRESS');
  }

  async readCurrent(length){
    if(length<1||length>256)throw new Error('READ length must be 1..256');
    const requested=length===256?0:length;
    const command=withCrc(Uint8Array.from([AM32.CMD_READ_FLASH,requested]));
    await this.t.sendWithEcho(command);

    const response=await this.t.readExact(length+3,1800);
    if(response[length+2]!==AM32.ACK_OK){
      throw new Error('READ_FLASH ACK '+hexByte(response[length+2]));
    }
    if(!checkDataCrc(response,length)){
      throw new Error('READ_FLASH data CRC mismatch');
    }
    return response.slice(0,length);
  }

  async readMemoryPhysical(physicalAddress,length){
    const result=new Uint8Array(length);
    let offset=0;
    while(offset<length){
      const chunk=Math.min(256,length-offset);
      await this.setAddressPhysical(physicalAddress+offset);
      const data=await this.readCurrent(chunk);
      result.set(data,offset);
      offset+=chunk;
    }
    return result;
  }

  async setBuffer(length){
    if(length<1||length>256)throw new Error('SET_BUFFER length must be 1..256');
    const command=withCrc(Uint8Array.from([
      AM32.CMD_SET_BUFFER,
      0x00,
      length===256?0x01:0x00,
      length===256?0x00:length
    ]));
    await this.t.sendWithEcho(command);

    // Official direct sequence has no normal ACK here. Only consume an
    // immediate error if the ESC rejected the SET_BUFFER command.
    const maybe=await this.t.readOptional(12);
    if(maybe===null)return;
    if(maybe===AM32.ACK_BAD_CRC)throw new Error('SET_BUFFER: BAD CRC (0xC2)');
    if(maybe===AM32.ACK_BAD_CMD)throw new Error('SET_BUFFER: BAD CMD (0xC1)');
    throw new Error('Unexpected response after SET_BUFFER: '+hexByte(maybe));
  }

  async sendPayload(payload){
    const framed=withCrc(payload);
    let lastError=null;

    for(let attempt=0;attempt<3;attempt++){
      if(attempt>0)await delay(20);
      try{
        await this.t.sendWithEcho(framed,2200);
        const ack=(await this.t.readExact(1,1200))[0];
        if(ack===AM32.ACK_OK)return;
        if(ack===AM32.ACK_BAD_CRC){
          lastError=new Error('Payload CRC rejected by ESC');
          continue; // bootloader remains in payload mode; retry same payload.
        }
        throw new Error('Payload ACK '+hexByte(ack));
      }catch(error){
        lastError=error;
        // A timeout is not safe to blindly retransmit because the bootloader
        // state is unknown. Retry only explicit C2 above.
        if(!String(error.message||error).includes('Payload CRC rejected'))throw error;
      }
    }
    throw lastError||new Error('Payload failed');
  }

  async programFlash(){
    await this.t.sendWithEcho(AM32.PROGRAM_FLASH_FRAME,1600);
    await this.readAck(1400,'PROGRAM_FLASH');
  }

  async writeBlockPhysical(physicalAddress,payload){
    if(payload.length<1||payload.length>AM32.BLOCK_SIZE){
      throw new Error(`Write block must be 1..${AM32.BLOCK_SIZE} bytes`);
    }
    await this.setAddressPhysical(physicalAddress);
    await this.setBuffer(payload.length);
    await this.sendPayload(payload);
    await this.programFlash();
  }

  async writeMemoryPhysical(physicalAddress,data,onProgress=null){
    let offset=0;
    while(offset<data.length){
      const length=Math.min(AM32.BLOCK_SIZE,data.length-offset);
      const block=data.slice(offset,offset+length);
      await this.writeBlockPhysical(physicalAddress+offset,block);
      offset+=length;
      if(onProgress)onProgress(offset,data.length);
      await delay(3);
    }
  }

  async runApplication(){
    const frame=Uint8Array.from([0x00,0x00,0x00,0x00]);
    // The ESP bridge intentionally waits ~250 ms before deciding four zeros
    // are RUN rather than a fragmented BootInit prefix.
    await this.t.sendWithEcho(frame,1600);
    await delay(80);
  }
}

function parseIntelHex(text,capacity){
  const image=new Uint8Array(capacity);
  image.fill(0x00); // Match the old ConfigTool/web conversion behavior.

  let addressBase=0;
  let mode=null; // 'app' or 'zero'
  let highest=0;
  let sawEof=false;

  const lines=text.replace(/\r/g,'').split('\n');
  for(let lineNumber=0;lineNumber<lines.length;lineNumber++){
    const line=lines[lineNumber].trim();
    if(!line)continue;
    if(line[0]!==':')throw new Error(`HEX line ${lineNumber+1}: missing ':'`);
    if(((line.length-1)&1)!==0)throw new Error(`HEX line ${lineNumber+1}: invalid length`);

    const bytes=[];
    for(let p=1;p<line.length;p+=2){
      const value=Number.parseInt(line.slice(p,p+2),16);
      if(!Number.isFinite(value))throw new Error(`HEX line ${lineNumber+1}: invalid hex`);
      bytes.push(value);
    }
    if(bytes.length<5)throw new Error(`HEX line ${lineNumber+1}: too short`);

    let sum=0;
    for(const b of bytes)sum=(sum+b)&0xFF;
    if(sum!==0)throw new Error(`HEX line ${lineNumber+1}: checksum error`);

    const count=bytes[0];
    if(bytes.length!==count+5)throw new Error(`HEX line ${lineNumber+1}: record length mismatch`);
    const addr=(bytes[1]<<8)|bytes[2];
    const type=bytes[3];
    const data=bytes.slice(4,4+count);

    if(type===0x00){
      let full=addressBase+addr;
      if(full>=0x08000000&&full<0x09000000)full-=0x08000000;

      if(mode===null)mode=full>=AM32.APP_START?'app':'zero';
      let outputOffset;
      if(mode==='app'){
        if(full<AM32.APP_START)throw new Error('HEX mixes zero-based and 0x1000 app addresses');
        outputOffset=full-AM32.APP_START;
      }else{
        outputOffset=full;
      }

      if(outputOffset+count>capacity){
        throw new Error('Firmware exceeds the application area before EEPROM');
      }
      image.set(data,outputOffset);
      highest=Math.max(highest,outputOffset+count);
    }else if(type===0x01){
      sawEof=true;
      break;
    }else if(type===0x02){
      if(count!==2)throw new Error('Invalid HEX segment-address record');
      addressBase=((data[0]<<8)|data[1])<<4;
    }else if(type===0x04){
      if(count!==2)throw new Error('Invalid HEX linear-address record');
      addressBase=((data[0]<<8)|data[1])*0x10000;
    }else if(type===0x03||type===0x05){
      // Start-address records are metadata; no flash payload is stored here.
    }else{
      throw new Error(`Unsupported HEX record type ${hexByte(type)}`);
    }
  }

  if(!sawEof)throw new Error('Intel HEX file has no EOF record');
  if(highest===0)throw new Error('Intel HEX contains no firmware data');
  return image.slice(0,highest);
}

class AM32WebApp {
  constructor(){
    this.transport=new SerialTransport();
    this.protocol=new AM32DirectProtocol(this.transport);
    this.eeprom=null;
    this.eepromRawAvailable=false;
    this.firmwareImage=null;
    this.flashTask=null;
    this.serialOperation=false;

    this.state={
      ok:true,
      connected:false,
      loaded:false,
      firmwareMissing:false,
      recovery:false,
      busy:false,
      message:'Not connected',
      mode:'USB DIRECT',
      wifiRunning:false,
      gpio:21,
      firmwareMajor:0,
      firmwareMinor:0,
      eepromVersion:0,
      eepromLength:0,
      eepromAddress:0,
      flashCode:0,
      pinCode:0,
      firmwareUploaded:false,
      firmwareSize:0,
      flashWritten:0,
      flashState:'idle',
      flashError:'',
      raw:null
    };

    if('serial' in navigator){
      navigator.serial.addEventListener('disconnect',event=>{
        if(this.transport.port&&event.target===this.transport.port){
          this._markDisconnected('USB device disconnected');
        }
      });
    }
  }

  _markDisconnected(message='Not connected'){
    this.state.connected=false;
    this.state.loaded=false;
    this.state.message=message;
  }

  statusSnapshot(includeSettings=false){
    const out={...this.state};
    if(includeSettings&&this.eeprom)out.raw=this._rawFromEeprom(this.eeprom);
    else if(this.state.raw)out.raw={...this.state.raw};
    return out;
  }

  _rawFromEeprom(bytes){
    const raw={};
    for(const [name,offset] of Object.entries(EEPROM_FIELDS))raw[name]=bytes[offset];
    return raw;
  }

  _validateEeprom(bytes){
    if(!bytes||bytes.length<AM32.EEPROM_BYTES)return false;
    if(bytes[0]!==0x00&&bytes[0]!==0x01)return false;
    if(bytes[3]===0xFF||bytes[4]===0xFF)return false;
    if(bytes[3]===0x00&&bytes[4]===0x00)return false;
    return true;
  }

  _displayEepromAddress(){
    if(this.protocol.flashCode===0x2B)return 0x7E00;
    return this.protocol.eepromPhysical&0xFFFF;
  }

  _loadMetadataFromEeprom(){
    if(!this.eeprom)return;
    this.state.eepromVersion=this.eeprom[1];
    this.state.firmwareMajor=this.eeprom[3];
    this.state.firmwareMinor=this.eeprom[4];
    this.state.eepromLength=AM32.EEPROM_BYTES;
    this.state.raw=this._rawFromEeprom(this.eeprom);
    this.state.recovery=this.eeprom[0]===0x00;
  }

  async _stableReadEeprom(){
    let last=null;
    for(let attempt=0;attempt<3;attempt++){
      const first=await this.protocol.readMemoryPhysical(this.protocol.eepromPhysical,AM32.EEPROM_BYTES);
      await delay(35);
      const second=await this.protocol.readMemoryPhysical(this.protocol.eepromPhysical,AM32.EEPROM_BYTES);
      if(bytesEqual(first,second))return second;
      last=second;
      await delay(60);
    }
    if(last)throw new Error('EEPROM read is unstable; check GPIO21/GND and reconnect');
    throw new Error('Cannot read EEPROM');
  }

  async _withSerialOperation(fn){
    if(this.serialOperation)throw new Error('Another ESC operation is already running');
    this.serialOperation=true;
    try{return await fn();}
    finally{this.serialOperation=false;}
  }

  async connectAndRead(){
    if(this.flashTask)throw new Error('Firmware installation is in progress');
    return this._withSerialOperation(async()=>{
      await this.transport.open();
      this.state.message='Connecting to AM32 bootloader';

      try{
        const info=await this.protocol.connect();
        this.state.connected=true;
        this.state.flashCode=info[4];
        this.state.pinCode=info[3];
        this.state.eepromAddress=this._displayEepromAddress();

        const ee=await this._stableReadEeprom();
        this.eeprom=ee;
        this.eepromRawAvailable=true;

        const valid=this._validateEeprom(ee);
        this.state.firmwareMissing=!valid;
        this.state.loaded=valid;
        this.state.recovery=valid?ee[0]===0x00:true;

        if(valid){
          this._loadMetadataFromEeprom();
          this.state.message=this.state.recovery
            ?'Bootloader connected · recovery mode'
            :'Device connected · Settings read OK';
        }else{
          this.state.raw=null;
          this.state.eepromLength=AM32.EEPROM_BYTES;
          this.state.message='Bootloader connected · firmware/settings area is not valid';
        }
        return this.statusSnapshot(true);
      }catch(error){
        this._markDisconnected('Connect failed');
        throw new Error((error&&error.message?error.message:String(error))+
          '. Power the ESP32-C3 first, power-cycle the ESC, then click Connect Device again.');
      }
    });
  }

  _applySettings(params,base){
    const out=Uint8Array.from(base);
    for(const [name,offset] of Object.entries(EEPROM_FIELDS)){
      if(params.has(name)){
        let value=Number(params.get(name));
        if(!Number.isFinite(value))continue;
        value=Math.max(0,Math.min(255,Math.round(value)));
        out[offset]=value;
      }
    }
    out[0]=0x01;
    return out;
  }

  async saveSettings(params){
    if(this.flashTask)throw new Error('Firmware installation is in progress');
    if(!this.state.connected||!this.state.loaded||!this.eeprom){
      throw new Error('Connect and read the ESC before saving Settings');
    }
    if(!(params instanceof URLSearchParams)){
      params=new URLSearchParams(params||'');
    }

    return this._withSerialOperation(async()=>{
      const original=await this._stableReadEeprom();
      if(!this._validateEeprom(original))throw new Error('EEPROM header is not valid; refusing to write Settings');
      const updated=this._applySettings(params,original);

      this.state.message='Writing Settings';
      await this.protocol.writeMemoryPhysical(this.protocol.eepromPhysical,updated);
      await delay(80);
      const verify=await this.protocol.readMemoryPhysical(this.protocol.eepromPhysical,AM32.EEPROM_BYTES);

      for(let i=0;i<AM32.EEPROM_BYTES;i++){
        // Bootloader versions can update byte 2 during a session; all user
        // settings and the boot byte must still verify exactly.
        if(i===2)continue;
        if(verify[i]!==updated[i]){
          throw new Error(`EEPROM verify failed at byte ${i}: wrote ${hexByte(updated[i])}, read ${hexByte(verify[i])}`);
        }
      }

      this.eeprom=verify;
      this.eepromRawAvailable=true;
      this.state.loaded=true;
      this.state.firmwareMissing=false;
      this._loadMetadataFromEeprom();
      this.state.message='Settings saved and verified';
      return this.statusSnapshot(true);
    });
  }

  async resetEsc(){
    if(this.flashTask)throw new Error('Firmware installation is in progress');
    if(!this.state.connected)throw new Error('ESC is not connected');
    return this._withSerialOperation(async()=>{
      await this.protocol.runApplication();
      this.state.connected=false;
      this.state.loaded=false;
      this.state.message='ESC run command sent';
      return this.statusSnapshot(false);
    });
  }

  _firmwareCapacity(){
    if(!this.protocol.eepromPhysical||this.protocol.eepromPhysical<=AM32.APP_START)return 0;
    return this.protocol.eepromPhysical-AM32.APP_START;
  }

  async uploadFirmware(formData){
    if(this.flashTask)throw new Error('Firmware installation is in progress');
    if(!this.state.connected)throw new Error('Connect the bootloader first');
    const file=formData&&typeof formData.get==='function'?formData.get('firmware'):null;
    if(!(file instanceof File))throw new Error('No firmware file selected');
    if(!/\.hex$/i.test(file.name))throw new Error('Select an Intel HEX (.hex) firmware file');

    const capacity=this._firmwareCapacity();
    if(!capacity)throw new Error('Unknown ESC flash capacity');
    const text=await file.text();
    this.firmwareImage=parseIntelHex(text,capacity);
    this.state.firmwareUploaded=true;
    this.state.firmwareSize=this.firmwareImage.length;
    this.state.flashWritten=0;
    this.state.flashState='ready';
    this.state.flashError='';
    this.state.message=`HEX ready · ${this.firmwareImage.length} bytes`;
    return this.statusSnapshot(true);
  }

  async startFlash(){
    if(this.flashTask)throw new Error('Firmware installation is already running');
    if(!this.state.connected)throw new Error('Bootloader is not connected');
    if(!this.firmwareImage||this.firmwareImage.length===0)throw new Error('No valid firmware has been loaded');
    if(!this.eepromRawAvailable||!this.eeprom){
      throw new Error('EEPROM could not be backed up. Reconnect the ESC before flashing.');
    }

    this.state.flashState='pending';
    this.state.flashWritten=0;
    this.state.flashError='';
    this.state.message='Preparing firmware';

    const task=this._flashFirmware();
    this.flashTask=task;
    task.catch(error=>{
      this.state.flashState='error';
      this.state.flashError=error&&error.message?error.message:String(error);
      this.state.message=this.state.flashError;
    }).finally(()=>{
      if(this.flashTask===task)this.flashTask=null;
    });

    return this.statusSnapshot(true);
  }

  async _flashFirmware(){
    return this._withSerialOperation(async()=>{
      const firmware=this.firmwareImage;
      const savedEeprom=Uint8Array.from(this.eeprom);

      // Safety byte = 0 while flashing, matching ConfigTool behavior.
      this.state.flashState='safety';
      this.state.message='Preparing device';
      const safety=Uint8Array.from(savedEeprom);
      safety[0]=0x00;
      await this.protocol.writeMemoryPhysical(this.protocol.eepromPhysical,safety);
      this.eeprom=Uint8Array.from(safety);

      this.state.flashState='writing';
      this.state.message='Installing firmware 0%';

      let offset=0;
      while(offset<firmware.length){
        const length=Math.min(AM32.BLOCK_SIZE,firmware.length-offset);
        const block=firmware.slice(offset,offset+length);
        await this.protocol.writeBlockPhysical(AM32.APP_START+offset,block);
        offset+=length;
        this.state.flashWritten=offset;
        const percent=Math.floor(offset*100/firmware.length);
        this.state.message=`Installing firmware ${percent}%`;
        await delay(3);
      }

      // Restore original EEPROM/settings and enable boot byte.
      this.state.flashState='finishing';
      this.state.message='Finalizing settings';
      const restored=Uint8Array.from(savedEeprom);
      restored[0]=0x01;
      await this.protocol.writeMemoryPhysical(this.protocol.eepromPhysical,restored);
      await delay(80);

      const verifyEe=await this.protocol.readMemoryPhysical(this.protocol.eepromPhysical,AM32.EEPROM_BYTES);
      for(let i=0;i<AM32.EEPROM_BYTES;i++){
        if(i===2)continue;
        if(verifyEe[i]!==restored[i])throw new Error(`Final EEPROM verify failed at byte ${i}`);
      }
      this.eeprom=verifyEe;
      this.eepromRawAvailable=true;
      this.state.flashWritten=firmware.length;

      this.state.flashState='reset';
      this.state.message='Firmware written · starting ESC';
      await this.protocol.runApplication();

      this.state.flashState='done';
      this.state.connected=false;
      this.state.loaded=false;
      this.state.firmwareMissing=false;
      this.state.recovery=false;
      this.state.message='Firmware installation complete';
    });
  }

  clearFirmware(){
    if(this.flashTask)throw new Error('Cannot clear firmware while flashing');
    this.firmwareImage=null;
    this.state.firmwareUploaded=false;
    this.state.firmwareSize=0;
    this.state.flashWritten=0;
    this.state.flashState='idle';
    this.state.flashError='';
    this.state.message=this.state.connected?'Firmware selection cleared':'Not connected';
    return this.statusSnapshot(true);
  }

  async api(url,options={}){
    switch(url){
      case '/api/status':
      case '/api/firmware/status':
        return this.statusSnapshot(true);

      case '/api/connect':
        return await this.connectAndRead();

      case '/api/save':
        return await this.saveSettings(options.body);

      case '/api/reset':
        return await this.resetEsc();

      case '/api/firmware/upload':
        return await this.uploadFirmware(options.body);

      case '/api/firmware/start':
        return await this.startFlash();

      case '/api/firmware/clear':
        return this.clearFirmware();

      case '/api/firmware/default':
        throw new Error('This USB browser build does not embed a default firmware. Choose your .hex file under Custom Firmware.');

      default:
        throw new Error('Unknown browser API route: '+url);
    }
  }
}

window.am32Web=new AM32WebApp();
