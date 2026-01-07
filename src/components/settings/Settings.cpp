#include "components/settings/Settings.h"
#include <cstdlib>
#include <cstring>

using namespace Pinetime::Controllers;

Settings::Settings(Pinetime::Controllers::FS& fs) : fs {fs} {
}

void Settings::Init() {

  // Load default settings from Flash
  LoadSettingsFromFile();
}

void Settings::SaveSettings() {

  // verify if is necessary to save
  if (settingsChanged) {
    SaveSettingsToFile();
  }
  settingsChanged = false;
}

void Settings::LoadSettingsFromFile() {
  SettingsData bufferSettings {};
  lfs_file_t settingsFile {};

  if (fs.FileOpen(&settingsFile, "/settings.dat", LFS_O_RDONLY) != LFS_ERR_OK) {
    return;
  }
  const int bytesRead = fs.FileRead(&settingsFile, reinterpret_cast<uint8_t*>(&bufferSettings), sizeof(bufferSettings));
  fs.FileClose(&settingsFile);

  if (bytesRead != static_cast<int>(sizeof(bufferSettings))) {
    fs.FileDelete("/settings.dat");
    return;
  }

  if (bufferSettings.version != settingsVersion) {
    fs.FileDelete("/settings.dat");
    return;
  }

  settings = bufferSettings;
}

void Settings::SaveSettingsToFile() {
  lfs_file_t settingsFile {};

  if (fs.FileOpen(&settingsFile, "/settings.dat", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) {
    return;
  }
  fs.FileWrite(&settingsFile, reinterpret_cast<uint8_t*>(&settings), sizeof(settings));
  fs.FileClose(&settingsFile);
}
