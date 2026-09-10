// NOTE: Ten interface tra ve cho App khi App gui cmd_InterfaceGetName.
#define SERIAL_4WAY_INTERFACE_NAME_STR "m4wFCIntf"
// *** change to adapt Revision
// NOTE: Ba gia tri nay tao version cua firmware interface, khong phai version firmware ESC.
#define SERIAL_4WAY_VER_MAIN 20
#define SERIAL_4WAY_VER_SUB_1 (uint8_t) 0
#define SERIAL_4WAY_VER_SUB_2 (uint8_t) 05

// NOTE: Version cua giao thuc 4Way. 108 decimal = 0x6C.
#define SERIAL_4WAY_PROTOCOL_VER 108

// NOTE: Ghep version thanh mot so, voi bo gia tri tren thi SERIAL_4WAY_VERSION = 20005.
#define SERIAL_4WAY_VERSION (uint16_t) ((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)

// NOTE: Tach version interface thanh 2 byte de tra ve cho App.
#define SERIAL_4WAY_VERSION_HI (uint8_t) (SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO (uint8_t) (SERIAL_4WAY_VERSION % 100)

// NOTE: Mode 4 la ARM BLHeli Bootloader, dung cho AM32/ARM trong code nay.
#define imARM_BLB 4


// Send Structure
// ESC + CMD PARAM_LEN [PARAM (if len > 0)] CRC16_Hi CRC16_Lo
// Return
// ESC CMD PARAM_LEN [PARAM (if len > 0)] + ACK (uint8_t OK or ERR) + CRC16_Hi CRC16_Lo
// NOTE: Frame request thuc te trong code:
// NOTE: [Local_Escape][CMD][ADDR_H][ADDR_L][LEN][PARAM...][CRC_H][CRC_L].
// NOTE: Frame response:
// NOTE: [Remote_Escape][CMD][ADDR_H][ADDR_L][LEN][DATA...][ACK][CRC_H][CRC_L].

// NOTE: 0x2E dung o byte dau cua frame ESP/interface tra ve App.
#define cmd_Remote_Escape 0x2E // '.'
// NOTE: 0x2F dung o byte dau cua moi frame 4Way App gui toi ESP/interface.
#define cmd_Local_Escape  0x2F // '/'

// Test Interface still present
// NOTE: Chi kiem tra interface/duong giao tiep con hoat dong.
#define cmd_InterfaceTestAlive 0x30 // '0' alive
// RETURN: ACK

// get Protocol Version Number 01..255
// NOTE: Hoi version protocol 4Way, ket qua la SERIAL_4WAY_PROTOCOL_VER.
#define cmd_ProtocolGetVersion 0x31  // '1' version
// RETURN: uint8_t VersionNumber + ACK

// get Version String
// NOTE: Hoi ten interface, vi du "m4wFCIntf".
#define cmd_InterfaceGetName 0x32 // '2' name
// RETURN: String + ACK

//get Version Number 01..255
// NOTE: Hoi version firmware cua interface, khong phai version ESC.
#define cmd_InterfaceGetVersion 0x33  // '3' version
// RETURN: uint8_t AVersionNumber + ACK


// Exit / Restart Interface - can be used to switch to Box Mode
// NOTE: Thoat che do 4Way/passthrough cua interface.
#define cmd_InterfaceExit 0x34       // '4' exit
// RETURN: ACK

// Reset the Device connected to the Interface
// NOTE: Yeu cau reset ESC dang ket noi voi interface.
#define cmd_DeviceReset 0x35        // '5' reset
// RETURN: ACK

// Get the Device ID connected
// #define cmd_DeviceGetID 0x36      //'6' device id removed since 06/106
// RETURN: uint8_t DeviceID + ACK

// Initialize Flash Access for Device connected
// NOTE: Bat tay voi bootloader ESC va lay thong tin chip/flash/mode.
#define cmd_DeviceInitFlash 0x37    // '7' init flash access
// RETURN: ACK

// Erase the whole Device Memory of connected Device
// NOTE: Lenh 4Way yeu cau xoa toan bo. Can co nhanh xu ly tuong ung trong Check_4Way().
#define cmd_DeviceEraseAll 0x38     // '8' erase all
// RETURN: ACK

// Erase one Page of Device Memory of connected Device
// NOTE: PARAM dau tien la so page can xoa.
#define cmd_DevicePageErase 0x39    // '9' page erase
// PARAM: uint8_t APageNumber
// RETURN: ACK
// Read to Buffer from Device Memory of connected Device // Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
// NOTE: Lenh doc memory 4Way. Check_4Way() se doi thanh CMD_SET_ADDRESS + CMD_READ_FLASH_SIL.
// Doc du lieu tu bo nho cua ESC vao buffer.
// Moi lan doc toi da 256 byte.
// BUFF_LEN = 0 duoc hieu la 256 byte.
// NOTE: Check_4Way() chuyen lenh nay thanh:
// NOTE: CMD_SET_ADDRESS + CMD_READ_FLASH_SIL.
#define cmd_DeviceRead 0x3A  // Lenh doc bo nho ESC
// NOTE: Khung gui:
// NOTE: [2F][3A][ADDR_H][ADDR_L][01][BUFF_LEN][CRC_H][CRC_L]
// NOTE: ADDR_H va ADDR_L la dia chi bat dau doc.
// NOTE: Byte 01 nghia la co mot byte parameter.
// NOTE: BUFF_LEN la so byte can doc.
// NOTE: Khung tra ve chua DATA + ACK + CRC.


// Ghi du lieu tu buffer vao bo nho cua ESC.
// Moi lan ghi toi da 256 byte.
// BUFF_LEN = 0 duoc hieu la 256 byte.
// NOTE: Check_4Way() tach lenh nay thanh:
// NOTE: SET_ADDRESS -> SET_BUFFER -> DATA -> PROG_FLASH.
#define cmd_DeviceWrite 0x3B  // Lenh ghi bo nho ESC
// NOTE: Khung gui:
// NOTE: [2F][3B][ADDR_H][ADDR_L][BUFF_LEN][DATA...][CRC_H][CRC_L]
// NOTE: ADDR_H va ADDR_L la dia chi bat dau ghi.
// NOTE: BUFF_LEN la so byte DATA.
// NOTE: Khung tra ve chua ACK bao thanh cong hoac loi.

// Giu chan C2CK o muc LOW de dat mot so chip vao trang thai reset.
// NOTE: Lenh nay dung cho chip giao tiep C2, vi du Silicon Labs.
// NOTE: AM32 dung chip ARM nen thong thuong khong dung lenh nay.
#define cmd_DeviceC2CK_LOW 0x3C
// NOTE: Ket qua tra ve la ACK.

// Doc EEPROM rieng cua mot so loai chip.
// Moi lan doc toi da 256 byte.
// BUFF_LEN = 0 duoc hieu la 256 byte.
#define cmd_DeviceReadEEprom 0x3D  // Lenh doc EEPROM rieng
// NOTE: Khung chua dia chi EEPROM va so byte can doc.
// NOTE: Ket qua tra ve gom du lieu EEPROM + ACK.
// NOTE CHECK: Check_4Way() hien tai chua co nhanh xu ly 0x3D.
// NOTE: AM32 luu cau hinh EEPROM trong Flash.
// NOTE: Vi vay AM32 thuong dung cmd_DeviceRead 0x3A de doc cau hinh.


// Ghi vao EEPROM rieng cua mot so loai chip.
// Moi lan ghi toi da 256 byte.
// BUFF_LEN = 0 duoc hieu la 256 byte.
#define cmd_DeviceWriteEEprom 0x3E  // Lenh ghi EEPROM rieng

// NOTE: Khung chua dia chi EEPROM, do dai va du lieu can ghi.
// NOTE: Ket qua tra ve la ACK.
// NOTE CHECK: Check_4Way() hien tai chua co nhanh xu ly 0x3E.
// NOTE: AM32 luu cau hinh EEPROM trong Flash.
// NOTE: Vi vay AM32 thuong dung cmd_DeviceWrite 0x3B de ghi cau hinh.
#define cmd_InterfaceSetMode 0x3F   // '?'
// #define imC2 0
// #define imSIL_BLB 1
// #define imATM_BLB 2
// #define imSK 3
// PARAM: uint8_t Mode
// RETURN: ACK or ACK_I_INVALID_CHANNEL
// NOTE: Code nay chi chap nhan imARM_BLB = 4.

//Write to Buffer for Verify Device Memory of connected Device //Buffer Len is Max 256 Bytes
//BuffLen = 0 means 256 Bytes
// NOTE: Yeu cau verify du lieu da ghi.
#define cmd_DeviceVerify 0x40   //'@' write
//PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
//RETURN: ACK
// NOTE CHECK: Nhanh DeviceVerify hien tai chi tra ACK, chua verify Flash thuc su.

// responses
// NOTE: Day la ACK cua lop 4Way, nam trong frame ESP/interface tra ve App.
#define ACK_OK                  0x00
// #define ACK_I_UNKNOWN_ERROR       0x01
#define ACK_I_INVALID_CMD       0x02
#define ACK_I_INVALID_CRC       0x03
#define ACK_I_VERIFY_ERROR      0x04
// #define ACK_D_INVALID_COMMAND 0x05
// #define ACK_D_COMMAND_FAILED  0x06
// #define ACK_D_UNKNOWN_ERROR       0x07

#define ACK_I_INVALID_CHANNEL   0x08
#define ACK_I_INVALID_PARAM     0x09
#define ACK_D_GENERAL_ERROR     0x0F


// Bootloader commands
// NOTE: Tu day tro xuong la command noi bo ESP/interface gui truc tiep toi bootloader ESC.
// NOTE: Cac frame nay khong bat dau bang 0x2F va dung CRC 0xA001, CRC_LOW truoc CRC_HIGH.
// RunCmd
// NOTE: Gia tri tham so dung khi reset/thoat bootloader trong code interface.
#define RestartBootloader   0
#define ExitBootloader      1

// NOTE: Thoat bootloader va chay firmware chinh.
#define CMD_RUN             0x00
// NOTE: Ghi payLoadBuffer vao dia chi Flash da dat truoc do.
#define CMD_PROG_FLASH      0x01
// NOTE: Yeu cau xoa Flash tai dia chi/page da chon.
#define CMD_ERASE_FLASH     0x02
// NOTE: Doc Flash; byte tiep theo la so byte doc, 0 nghia la 256 byte.
#define CMD_READ_FLASH_SIL  0x03
// NOTE: Cung ma 0x03 nhung y nghia tuy target/mode protocol.
#define CMD_VERIFY_FLASH    0x03
// NOTE: Cung ma 0x04 nhung y nghia tuy target/mode protocol.
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM     0x04
#define CMD_PROG_EEPROM     0x05
#define CMD_READ_SRAM       0x06
#define CMD_READ_FLASH_ATM  0x07
// NOTE: Kiem tra/giu bootloader khong tu thoat ve firmware chinh.
#define CMD_KEEP_ALIVE      0xFD
// NOTE: Frame: [FF][ADDR_H][ADDR_L][CRC_L][CRC_H].
#define CMD_SET_ADDRESS     0xFF
// NOTE: Frame: [FE][SIZE_H][SIZE_L][CRC_L][CRC_H]. SIZE_H=1,SIZE_L=0 la 256 byte.
#define CMD_SET_BUFFER      0xFE

// NOTE: Command handshake/signature cua bootloader, khac command 4Way ben tren.
#define CMD_BOOTINIT        0x07
#define CMD_BOOTSIGN        0x08

// Bootloader result codes
// NOTE: Day la ket qua truc tiep cua bootloader ESC, khac ACK_* cua lop 4Way.
#define brSUCCESS           0x30
#define brERRORVERIFY       0xC0
#define brERRORCOMMAND      0xC1
#define brERRORCRC          0xC2
#define brNONE              0xFF


// NOTE: Nhan frame 4Way 0x2F, kiem tra CRC, dich sang command bootloader va tao response 0x2E.
uint16_t Check_4Way(uint8_t buf[]);
// NOTE: Tinh CRC-XMODEM 0x1021 cho frame 4Way, khong dung cho frame bootloader ESC.
uint16_t _crc_xmodem_update (uint16_t crc, uint8_t data);
