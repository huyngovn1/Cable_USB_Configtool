from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import os
import webbrowser

ROOT = Path(__file__).resolve().parent
os.chdir(ROOT)
address = ('127.0.0.1', 8765)
url = 'http://localhost:8765/index.html'

print('VPTEK AM32 Web Serial Configurator')
print('Open:', url)
print('Use desktop Chrome or Edge.')
webbrowser.open(url)
ThreadingHTTPServer(address, SimpleHTTPRequestHandler).serve_forever()
