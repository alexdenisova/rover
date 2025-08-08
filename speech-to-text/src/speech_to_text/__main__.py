import logging
import os
import re
import shutil
from pathlib import Path

import uvicorn
from fastapi import FastAPI, Header, HTTPException, UploadFile, status
from fastapi.responses import PlainTextResponse
from faster_whisper import WhisperModel

app = FastAPI()

# Configuration
CHUNKS_DIR = os.getenv("S2T_CHUNKS_DIR", "/tmp/chunks")
AUDIO_DIR = os.getenv("S2T_AUDIO_DIR", "/tmp")
AUDIO_PATH = Path(AUDIO_DIR) / "audio.wav"

# Ensure directories exist
Path(CHUNKS_DIR).mkdir(parents=True, exist_ok=True)
Path(AUDIO_DIR).mkdir(parents=True, exist_ok=True)

# Load model
model = WhisperModel("tiny", device="cpu", compute_type="int8")


@app.post("/transcribe")
async def transcribe(file: UploadFile, content_range: str = Header(None)):
    try:
        if content_range:
            # Handle chunked upload
            range_match = re.match(r"bytes (\d+)-(\d+)/(\d+)", content_range)
            if not range_match:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Invalid Content-Range format",
                )

            start = int(range_match.group(1))
            end = int(range_match.group(2))
            total_size = int(range_match.group(3))
            chunk_size = end - start + 1

            # Save chunk
            chunk_path = Path(CHUNKS_DIR) / f"{start}-{end}"
            received_size = await save_upload_file(file, chunk_path)

            if received_size != chunk_size:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Chunk has unexpected size. Expected {chunk_size}, received {received_size}",
                )

            # Check if this is the last chunk
            if end == (total_size - 1):
                combine_chunks(AUDIO_PATH, CHUNKS_DIR)
                logging.info(f"Combined chunks as {AUDIO_PATH}")
            else:
                return PlainTextResponse(
                    "Chunk received", status_code=status.HTTP_200_OK
                )
        else:
            # Handle single file upload
            received_size = await save_upload_file(file, AUDIO_PATH)
            if received_size == 0:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST, detail="No file uploaded"
                )

        # Transcribe the audio
        segments, info = model.transcribe(str(AUDIO_PATH))
        logging.info(
            "Detected language '%s' with probability %f",
            info.language,
            info.language_probability,
        )

        text = "".join(segment.text for segment in segments).strip()

        # Clean up
        AUDIO_PATH.unlink(missing_ok=True)

        return PlainTextResponse(text)

    except Exception as e:
        AUDIO_PATH.unlink(missing_ok=True)
        logging.error(f"Transcription error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error during transcription: {str(e)}",
        )

    def download_file(self, file_path: str):
        """Downloads file from form. Returns number of bytes downloaded."""
        form = cgi.FieldStorage(
            fp=self.rfile, headers=self.headers, environ={"REQUEST_METHOD": "POST"}
        )
        if "file" not in form:
            return 0
        file_item = form["file"]
        file_content = file_item.file.read()

        with open(file_path, "wb") as file:
            file.write(file_content)
        logging.info(f"Downloaded file to {file_path}")
        return os.path.getsize(file_path)

    def combine_chunks(self, final_path, chunks_dir):
        """Combine all chunks into the final file."""
        chunks = sorted(
            [
                os.path.join(chunks_dir, f)
                for f in os.listdir(chunks_dir)
                if os.path.isfile(os.path.join(chunks_dir, f))
            ]
        )

        with open(final_path, "wb") as final_file:
            for chunk_path in chunks:
                with open(chunk_path, "rb") as chunk_file:
                    final_file.write(chunk_file.read())
                os.remove(chunk_path)  # Delete the chunk after combining


@app.get("/health")
async def health_check():
    return PlainTextResponse("OK")


async def save_upload_file(upload_file: UploadFile, destination: Path) -> int:
    """Save uploaded file to destination and return size in bytes"""
    try:
        with destination.open("wb") as buffer:
            content = await upload_file.read()
            buffer.write(content)
        logging.info(f"Saved file to {destination}")
        return destination.stat().st_size
    except Exception as e:
        logging.error(f"Error saving file: {str(e)}")
        raise


def combine_chunks(final_path: Path, chunks_dir: str):
    """Combine all chunks into final file"""
    chunks = sorted(
        Path(chunks_dir) / f
        for f in os.listdir(chunks_dir)
        if (Path(chunks_dir) / f).is_file()
    )

    with final_path.open("wb") as final_file:
        for chunk_path in chunks:
            with chunk_path.open("rb") as chunk_file:
                final_file.write(chunk_file.read())
            chunk_path.unlink()  # Delete the chunk


if __name__ == "__main__":
    if Path(CHUNKS_DIR).exists():
        shutil.rmtree(CHUNKS_DIR)
    Path(CHUNKS_DIR).mkdir(mode=0o755)

    # Configure logging
    logging.basicConfig(
        format="%(asctime)s - %(levelname)s: %(message)s", level=logging.DEBUG
    )

    uvicorn.run(
        "speech_to_text.__main__:app",
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="info",
    )
