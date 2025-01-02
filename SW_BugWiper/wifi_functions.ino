#if USE_WIFI
// the following is based on https://github.com/smford/esp32-asyncwebserver-fileupload-example.git

void check_wifi_functions(void){
      // reboot if we've told it to reboot
    if (shouldReboot) {
      rebootESP("Web Admin Initiated Reboot");
    }
    if (shouldUpdate) {
      updateESP("Web Admin Initiated Update");
    }
}

void rebootESP(String message) {
  Serial.print("Rebooting ESP32: ");
  Serial.println(message);
  ESP.restart();
}

void updateESP(String message) {
  Serial.print("Rebooting ESP32: ");
  Serial.println(message);
  File firmware = FFat.open("/firmware.bin");
  if (firmware) {
    Serial.println(F("found!"));
    Serial.println(F("Try to update!"));

    //Update.onProgress(progressHandler);
    Update.begin(firmware.size(), U_FLASH);
    Update.writeStream(firmware);
    if (Update.end()) {
      Serial.println(F("Update finished!"));
    } else {
      Serial.println(F("Update error!"));
      Serial.println(Update.getError());
    }

    firmware.close();

    if (FFat.rename("/firmware.bin", "/firmware.bak")) {
      Serial.println(F("Firmware rename succesfully!"));
    } else {
      Serial.println(F("Firmware rename error!"));
    }
    delay(2000);

    ESP.restart();
  }
}

// list all of the files, if ishtml=true, return html rather than simple text
String listFiles(bool ishtml) {
  String returnText = "";
  Serial.println("Listing files stored on FatFS");
  File root = FFat.open("/");
  File foundfile = root.openNextFile();
  if (ishtml) {
    returnText += "<table><tr><th align='left'>Name</th><th align='left'>Size</th><th></th><th></th></tr>";
  }
  while (foundfile) {
    if (ishtml) {
      returnText += "<tr align='left'><td>" + String(foundfile.name()) + "</td><td>" + humanReadableSize(foundfile.size()) + "</td>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'download\')\">Download</button>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'delete\')\">Delete</button></tr>";
    } else {
      returnText += "File: " + String(foundfile.name()) + " Size: " + humanReadableSize(foundfile.size()) + "\n";
    }
    foundfile = root.openNextFile();
  }
  if (ishtml) {
    returnText += "</table>";
  }
  root.close();
  foundfile.close();
  return returnText;
}

// Make size of files human readable
// source: https://github.com/CelliesProjects/minimalUploadAuthESP32
String humanReadableSize(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " kB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}
#endif