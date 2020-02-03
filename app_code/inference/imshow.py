from flask import Flask, render_template

import asyncio
import websockets
from websockets.exceptions import ConnectionClosed

from base64 import b64encode
import cv2
import threading
# import os

# _DEFAULT_IMG = cv2.imread(os.path.join(
#     os.path.dirname(__file__), 'templates', 'Roboy_chef.jpg'))

# if _DEFAULT_IMG is None:
#     raise ValueError("Default image was not found")



def cvImgToBase64(img):
    r, buffer = cv2.imencode('.jpg', img)
    encoded = b64encode(buffer)
    encoded = "data:image/jpeg;charset=utf-8;base64," + str(encoded)[2:-1]
    return encoded


class ImageServer:
    def __init__(self):
        self.current_img = None
        self._current_text = ""

        self.clients = set()

        ip = '0.0.0.0'
        ws_server = websockets.serve(self.serve, ip, 8765, ping_timeout=60)
        loop = asyncio.get_event_loop()
        loop.run_until_complete(ws_server)
        self.wst = threading.Thread(target=loop.run_forever)
        self.wst.daemon = True

        app = Flask(__name__, static_folder="frontend_files/static", template_folder="frontend_files")

        @app.route('/')
        def index():
            """Serve the index HTML"""
            return render_template("index.html")

        self.flask_app = threading.Thread(
            target=app.run, kwargs={'port': 8000, 'host': ip, 'debug': False})
        self.flask_app.daemon = True
        self.wst.start()
        self.flask_app.start()

    def stop(self):
        self.flask_app.stop()
        self.wst.stop()

    async def serve(self, websocket, path):
        self.clients.add(websocket)
        await websocket.wait_closed()
        self.clients.remove(websocket)

    async def broadcastAsync(self, msg):
        clients = list(self.clients)
        for client in clients:
            try:
                await client.send(str(msg))
            except Exception as e:
                print(e)
                print(msg)

    def broadcast(self, msg):
        asyncio.new_event_loop().run_until_complete(self.broadcastAsync(msg))        
        # asyncio.run(self.broadcastAsync(msg))

    @property
    def current_text(self):
        return self._current_text

    @current_text.setter
    def current_text(self, val):
        self._current_text = str( val )
        self.broadcast("3" + self._current_text)

    # show opencv image
    def showImg(self, img):
        encoded = cvImgToBase64(img)
        self.current_img = encoded
        self.broadcast("0" + self.current_img)

    # def showDefault(self):
    #     self.current_img = _DEFAULT_IMG

    def progressStart(self):
        self.broadcast("1" + self.current_img)
