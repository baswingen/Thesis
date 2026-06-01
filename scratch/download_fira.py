import os
import urllib.request
from pathlib import Path

def download_file(url, dest_path):
    print(f"Downloading {url} -> {dest_path}")
    try:
        urllib.request.urlretrieve(url, dest_path)
        print("Success!")
    except Exception as e:
        print(f"Failed to download: {e}")

def main():
    base_dir = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis")
    fonts_dir = base_dir / "visualization" / "fonts"
    fonts_dir.mkdir(parents=True, exist_ok=True)
    
    font_urls = {
        "FiraSans-Regular.ttf": "https://github.com/google/fonts/raw/main/ofl/firasans/FiraSans-Regular.ttf",
        "FiraSans-Medium.ttf": "https://github.com/google/fonts/raw/main/ofl/firasans/FiraSans-Medium.ttf",
        "FiraSans-SemiBold.ttf": "https://github.com/google/fonts/raw/main/ofl/firasans/FiraSans-SemiBold.ttf",
        "FiraSans-Bold.ttf": "https://github.com/google/fonts/raw/main/ofl/firasans/FiraSans-Bold.ttf",
        "FiraSans-Italic.ttf": "https://github.com/google/fonts/raw/main/ofl/firasans/FiraSans-Italic.ttf",
        "FiraSans-MediumItalic.ttf": "https://github.com/google/fonts/raw/main/ofl/firasans/FiraSans-MediumItalic.ttf",
        "FiraSans-SemiBoldItalic.ttf": "https://github.com/google/fonts/raw/main/ofl/firasans/FiraSans-SemiBoldItalic.ttf",
        "FiraSans-BoldItalic.ttf": "https://github.com/google/fonts/raw/main/ofl/firasans/FiraSans-BoldItalic.ttf"
    }
    
    for filename, url in font_urls.items():
        dest = fonts_dir / filename
        if dest.exists():
            print(f"{filename} already exists, skipping.")
        else:
            download_file(url, dest)

if __name__ == "__main__":
    main()
