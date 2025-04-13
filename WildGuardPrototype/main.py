from fastapi import FastAPI, File, UploadFile
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import shutil
import os

app = FastAPI()

UPLOAD_FOLDER = "uploaded_files"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

app.mount("/uploaded_files", StaticFiles(directory=UPLOAD_FOLDER), name="uploaded_files")

@app.get("/", response_class=HTMLResponse)
def form():
      return """
    <form action="/upload" method="post" enctype="multipart/form-data">
        <input type="file" name="file">
        <button type="submit">Wyślij</button>
    </form>
    """

@app.post("/upload", response_class=HTMLResponse)
async def upload(file: UploadFile = File(...)):
    path = os.path.join(UPLOAD_FOLDER, file.filename)
    with open(path, "wb") as buffer:
        shutil.copyfileobj(file.file, buffer)

    return f"""
    <h2>Obrazek przesłany pomyślnie!</h2>
    <img src="/uploaded_files/{file.filename}" style="max-width:500px;"><br>
    <a href="/">Wyślij kolejny</a>
    """
