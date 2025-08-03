import cgi
import os
import tempfile
from http.server import BaseHTTPRequestHandler, HTTPServer

import whisper

# Load the Whisper model (do this once at startup)
model = whisper.load_model("tiny")  # You can use "small", "medium", etc.


class WhisperHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == "/transcribe":
            # Parse the multipart form data
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
                    # Save the uploaded content to a temporary file
                    file_content = file_item.file.read()
                    temp_audio.write(file_content)
                    temp_path = temp_audio.name

                # Transcribe using Whisper
                result = model.transcribe(temp_path)

                # Clean up the temporary file
                os.unlink(temp_path)

                # Send the transcription result
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.end_headers()
                self.wfile.write(result["text"].encode("utf-8"))

            except Exception as e:
                # Clean up if temp file was created
                if "temp_path" in locals() and os.path.exists(temp_path):
                    os.unlink(temp_path)

                self.send_response(500)
                self.end_headers()
                self.wfile.write(
                    f"Error during transcription: {str(e)}".encode("utf-8")
                )


def run(server_class=HTTPServer, handler_class=WhisperHandler, port=8000):
    server_address = ("", port)
    httpd = server_class(server_address, handler_class)
    print(f"Starting Whisper server on port {port}...")
    httpd.serve_forever()


if __name__ == "__main__":
    run()
