import os
import urllib.request
import tarfile

def download_and_extract_tum(url, download_dir):
    os.makedirs(download_dir, exist_ok=True)
    filename = url.split('/')[-1]
    file_path = os.path.join(download_dir, filename)
    
    # Download
    print(f"Скачивание {filename}...")
    # Add User-Agent header to avoid 403 Forbidden
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req) as response, open(file_path, 'wb') as out_file:
        data = response.read()
        out_file.write(data)
    
    print(f"Скачивание завершено. Файл сохранен в: {file_path}")
    
    # Extract
    print(f"Распаковка {filename}...")
    with tarfile.open(file_path, "r:gz") as tar:
        tar.extractall(path=download_dir)
        
    print(f"Распаковка завершена. Датасет готов в папке: {download_dir}")

if __name__ == "__main__":
    # URL для датасета fr1_desk (стандартная сцена с объектами на столе, идеальна для MVP)
    TUM_URL = "https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz"
    TARGET_DIR = "./datasets"
    
    download_and_extract_tum(TUM_URL, TARGET_DIR)
