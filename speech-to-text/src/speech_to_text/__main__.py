import cgi
import os
import tempfile
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer

from faster_whisper import WhisperModel

model = WhisperModel("tiny", device="cpu", compute_type="int8")


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == "/transcribe":
            # Get audio file from form
            form = cgi.FieldStorage(
                fp=self.rfile, headers=self.headers, environ={"REQUEST_METHOD": "POST"}
            )
            if "file" not in form:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b"No audio file uploaded")
                return
            file_item = form["file"]

            try:
                # Create a temporary file to store the audio
                with tempfile.NamedTemporaryFile(
                    delete=False, suffix=".wav"
                ) as temp_audio:
                    file_content = file_item.file.read()
                    temp_audio.write(file_content)
                    temp_path = temp_audio.name

                # Transcribe using Faster Whisper
                segments, info = model.transcribe(temp_path)
                print(
                    "Detected language '%s' with probability %f"
                    % (info.language, info.language_probability)
                )
                text = ""
                for segment in segments:
                    text += segment.text
                text = text.strip()

                # Delete the temporary file
                os.unlink(temp_path)

                # Send the transcription result
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.end_headers()
                self.wfile.write(text.encode("utf-8"))

            except Exception as e:
                # Delete the temp file if created
                if "temp_path" in locals() and os.path.exists(temp_path):
                    os.unlink(temp_path)

                self.send_response(500)
                self.end_headers()
                self.wfile.write(
                    f"Error during transcription: {str(e)}".encode("utf-8")
                )
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        if self.path == "/health":
            self.send_response(200)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_request(self, code="-", size="-"):
        if self.path == "/health":
            return
        if isinstance(code, HTTPStatus):
            code = code.value
        self.log_message('"%s" %s %s', self.requestline, str(code), str(size))


def run(server_class=HTTPServer, handler_class=Handler, port=8000):
    server_address = ("", port)
    httpd = server_class(server_address, handler_class)
    print(f"Starting server on port {port}...")
    httpd.serve_forever()


if __name__ == "__main__":
    run()
