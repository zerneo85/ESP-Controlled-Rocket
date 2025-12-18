// === sd_read_write.cpp ===
#include "sd_read_write.h"

// Used as a global verbosity control from the main firmware.
// This allows this module to stay quiet unless explicitly enabled.
extern bool debugSerial;

// ----------------------------------------------------------------------------
// copyFileFS()
// ----------------------------------------------------------------------------
// Cross-filesystem file copy helper.
//
// Typical usage (your firmware):
//   copyFileFS(SPIFFS, "/log.csv", SD_MMC, "/backup.csv");
//
// Notes:
// - Uses a fixed 512-byte buffer for moderate performance and low RAM usage.
// - Ensures dst.flush() before closing for safer writes on removable media.
// ----------------------------------------------------------------------------
bool copyFileFS(fs::FS &srcFS, const char *srcPath, fs::FS &dstFS, const char *dstPath) {
  File src = srcFS.open(srcPath, FILE_READ);
  if (!src) return false;

  File dst = dstFS.open(dstPath, FILE_WRITE);
  if (!dst) { src.close(); return false; }

  uint8_t buf[512];
  size_t n;
  while ((n = src.read(buf, sizeof(buf))) > 0) {
    if (dst.write(buf, n) != n) {
      src.close();
      dst.close();
      return false;
    }
  }

  dst.flush();
  src.close();
  dst.close();
  return true;
}

// Simple wrapper, mostly for readability.
bool fileExists(fs::FS &fs, const char *path) {
  return fs.exists(path);
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) Serial.println("Dir created");
  else Serial.println("mkdir failed");
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) Serial.println("Dir removed");
  else Serial.println("rmdir failed");
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (file.print(message)) Serial.println("File written");
  else Serial.println("Write failed");
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  if (debugSerial) Serial.printf("Appending to file: %s\n", path);

  // Most cores support FILE_APPEND; if it fails, some platforms require FILE_WRITE and seek.
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    // Fallback strategy: open as write and seek to end.
    file = fs.open(path, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for appending");
      return;
    }
    file.seek(file.size());
  }

  if (file.print(message)) {
    if (debugSerial) Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }

  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) Serial.println("File renamed");
  else Serial.println("Rename failed");
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) Serial.println("File deleted");
  else Serial.println("Delete failed");
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;

  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) toRead = 512;
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\r\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}
