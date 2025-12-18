// === sd_read_write.h ===
#ifndef __SD_READ_WRITE_H
#define __SD_READ_WRITE_H

#include "Arduino.h"
#include "FS.h"

// This module contains filesystem helpers that work with any FS implementation
// (SPIFFS, SD, SD_MMC, LittleFS, etc.), as long as it exposes the Arduino FS API.

// Directory & file helpers
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);

// Robust append that falls back to FILE_WRITE + seek if FILE_APPEND fails.
// NOTE: Some FS implementations / Arduino core variants have quirky append semantics.
void appendFile(fs::FS &fs, const char *path, const char *message);

void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);

// Quick IO test
void testFileIO(fs::FS &fs, const char *path);

// Robust cross-FS copy: from:/srcPath → to:/dstPath
// Returns true on success, false on any open/read/write failure.
bool copyFileFS(fs::FS &from, const char *srcPath, fs::FS &to, const char *dstPath);

// Convenience wrapper
bool fileExists(fs::FS &fs, const char *path);

#endif
