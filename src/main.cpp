
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*





                                                  
                                                  Adi's Doorlock





*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "esp_system.h"
#include "esp_wifi_types.h"
#include "esp_chip_info.h"
#include "esp_wifi.h"

#include <TFT_eSPI.h>
#include <SD.h>

#include "stdlib_noniso.h"
#include <functional>
#include "Update.h"
#include "StreamString.h"
#include <Preferences.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "time.h"
#include "esp_sntp.h"
#include <pgmspace.h>

#include <Adafruit_NeoPixel.h>

#include <anyrtttl.h>
#include <binrtttl.h>
#include <pitches.h>

#include <SparkFunSX1509.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_AHTX0.h>

#include "credentials.h"
#include "definitions.h"


String HTML_Header();
void Handle_Home(AsyncWebServerRequest *request);
void Handle_Wifi(AsyncWebServerRequest * request);
void Handle_Wifi_List(AsyncWebServerRequest *request);
void Handle_Wifi_Save(AsyncWebServerRequest * request);
void connectToNewWiFi();
int countFilesInDirectory(String path);
int getFileTypePriority(String filename, String ftype);
void Handle_SD_Dir(AsyncWebServerRequest * request);
void SD_Directory(String path);
void Handle_SD_File_Upload(AsyncWebServerRequest *request);
void on_SD_File_Upload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final);
void createDirectoryRecursive(const String& path);
void Handle_SD_File_Download(AsyncWebServerRequest *request);
void deleteRecursive(String path);
void Handle_SD_File_Delete(AsyncWebServerRequest *request);
void Handle_SD_File_Rename(AsyncWebServerRequest *request);
void Handle_SD_File_Move(AsyncWebServerRequest *request);
void Handle_Create_Folder(AsyncWebServerRequest *request);
void Display_System_Info(AsyncWebServerRequest *request);
void Handle_OTA(AsyncWebServerRequest *request);
void Handle_Page_Not_Found(AsyncWebServerRequest *request);
String ConvBinUnits(uint64_t bytes, int resolution);
String EncryptionType(wifi_auth_mode_t encryptionType);
void WIFI_CopySSIDs(wifi_ssid_count_t n);
void WIFI_Connect(String ssid, String pass);
bool WIFI_Scan();
void Webserver_Init();

void get_eeprom();
void update_eeprom();
void Matrix_Handler();
void printLog(String log);
void setup();
void loop();

Preferences preferences;
AsyncWebServer server(80);
DNSServer DNS;
TFT_eSPI tft = TFT_eSPI();
SPIClass sdSPI(VSPI);
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_Leds, NEO_GRB + NEO_KHZ800);
Adafruit_AHTX0 AHT;
SX1509 ExtraIO;  
Adafruit_VL53L0X LIDAR = Adafruit_VL53L0X();

//========================================================================================================================================
//======================= WiFi and Webserver Functions ========================
//========================================================================================================================================

String HTML_Header() {

  String wifi_button;

  if(wifi_status != WL_CONNECTED) wifi_button = "<a href='/wifi'>Wi-Fi</a>";

  return R"rawliteral(
  <!DOCTYPE html>
  <html lang='en'>
  <head>
    <title>Adi-Cam</title>
    <meta charset='UTF-8'>
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>
    <style>

    body {
      opacity: 0;
      transition: opacity 0.4s ease-in-out;
      max-width: 75em;
      margin: auto;
      font-family: Arial, Helvetica, sans-serif;
      font-size: 16px;
      color: #eeeeee;
      text-align: center;

      background-image: url('/background_)rawliteral" + String(chosen_background) + R"rawliteral('); 
      background-size: cover;
      background-repeat: no-repeat;
      background-position: center;
      background-attachment: fixed;
    }

    body.fade-in {
      opacity: 1;
    }

    .topnav {
      background: rgba(38, 38, 38, 0.5);
      backdrop-filter: blur(5px);
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.5);
      border-radius: 1em;
      
      padding: 0.5em 1em;
      margin: 1em auto;
      width: fit-content;
      
      color: white;
      display: flex;
      justify-content: center;
      flex-wrap: wrap;
      gap: 1em;
      margin-bottom: 2em;
    }
    
    .topnav a {
      color: white;
      background: rgba(255, 255, 255, 0.1);
      padding: 0.75em 1.2em;
      text-decoration: none;
      font-size: 1.1em;
      transition: background 0.3s ease;
      border-radius: 0.75em;
      font-weight: bold;
    }
    
    .topnav a:hover {
      background-color: rgba(255, 255, 255, 0.3);
      transform: scale(1.02);
    }

    .popup-overlay {
      display: none;
      position: fixed;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      background: rgba(0, 0, 0, 0.7);
      z-index: 999;
      animation: fadeIn 0.3s ease-out;
    }
    
    .popup-overlay.show {
      display: flex;
      justify-content: center;
      align-items: center;
    }
    
    .popup-content {
      background: linear-gradient(135deg, rgba(60, 60, 80, 0.95), rgba(40, 40, 60, 0.95));
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 1.5em;
      padding: 2.5em;
      max-width: 420px;
      min-width: 320px;
      text-align: center;
      backdrop-filter: blur(10px);
      box-shadow: 0 10px 40px rgba(0, 0, 0, 0.9);
      animation: slideUp 0.3s ease-out;
    }
    
    .popup-content h2 {
      color: #ffffff;
      margin: 0 0 1.2em 0;
      font-size: 1.5em;
      font-weight: 600;
      text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
      background: linear-gradient(135deg, #fff, #ccc);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
    }
    
    .popup-content p {
      color: #e8e8e8;
      margin: 0 0 2em 0;
      font-size: 1em;
      line-height: 1.5;
    }
    
    .popup-buttons {
      display: flex;
      gap: 1em;
      margin-bottom: 0;
      justify-content: center;
    }
    
    .popup-btn {
      padding: 0.85em 2em;
      border: none;
      border-radius: 0.75em;
      font-size: 1em;
      cursor: pointer;
      transition: all 0.3s ease;
      font-weight: bold;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
    }
    
    .popup-btn-confirm {
      background: linear-gradient(135deg, #00c6ff, #0072ff);
      color: white;
    }
    
    .popup-btn-confirm:hover:not(:disabled) {
      background: linear-gradient(135deg, #00d4ff, #0080ff);
      transform: translateY(-2px);
      box-shadow: 0 6px 16px rgba(0, 114, 255, 0.4);
    }
    
    .popup-btn-confirm:disabled {
      opacity: 0.6;
      cursor: not-allowed;
    }
    
    .popup-btn-cancel {
      background: rgba(255, 255, 255, 0.15);
      color: #ffffff;
    }
    
    .popup-btn-cancel:hover:not(:disabled) {
      background: rgba(255, 255, 255, 0.25);
      transform: translateY(-2px);
      box-shadow: 0 6px 16px rgba(255, 255, 255, 0.2);
    }
    
    .popup-btn-cancel:disabled {
      opacity: 0.6;
      cursor: not-allowed;
    }
    
    .popup-loading {
      display: none;
      margin-top: 1.5em;
      padding-top: 1.5em;
      border-top: 1px solid rgba(255, 255, 255, 0.1);
      color: #c8c8c8;
      font-size: 0.95em;
    }
    
    .popup-loading.show {
      display: block;
    }

    .spinner {
      display: inline-block;
      width: 16px;
      height: 16px;
      border: 2px solid rgba(255, 255, 255, 0.3);
      border-top-color: #00c6ff;
      border-radius: 50%;
      animation: spin 0.8s linear infinite;
      margin-right: 0.5em;
      vertical-align: middle;
    }
    
    @keyframes fadeIn {
      from { opacity: 0; }
      to { opacity: 1; }
    }
    
    @keyframes slideUp {
      from { 
        opacity: 0;
        transform: translateY(40px) scale(0.95);
      }
      to { 
        opacity: 1;
        transform: translateY(0) scale(1);
      }
    }

    @keyframes spin {
      to { transform: rotate(360deg); }
    }

    </style>
  </head>
  <body>
    <div id="popupOverlay" class="popup-overlay">
      <div class="popup-content">
        <h2 id="popupTitle">Confirm Action</h2>
        <p id="popupMessage">Are you sure?</p>
        <div class="popup-buttons">
          <button class="popup-btn popup-btn-confirm" id="popupConfirm">Confirm</button>
          <button class="popup-btn popup-btn-cancel" id="popupCancel">Cancel</button>
        </div>
        <div class="popup-loading" id="popupLoading">
          <span class="spinner"></span>
          Processing...
        </div>
      </div>
    </div>

    <script>
      let currentAction = null;

      function showPopup(action, title, message) {
        currentAction = action;
        document.getElementById('popupTitle').textContent = title;
        document.getElementById('popupMessage').textContent = message;
        document.getElementById('popupOverlay').classList.add('show');
        document.getElementById('popupLoading').classList.remove('show');
        document.getElementById('popupConfirm').style.display = 'block';
        document.getElementById('popupCancel').textContent = 'Cancel';
      }

      function closePopup() {
        document.getElementById('popupOverlay').classList.remove('show');
        currentAction = null;
      }

      document.getElementById('popupConfirm').addEventListener('click', async function() {
        if (!currentAction) return;
        
        document.getElementById('popupLoading').classList.add('show');
        document.getElementById('popupConfirm').disabled = true;
        document.getElementById('popupCancel').disabled = true;
        
        try {
          const response = await fetch(currentAction, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'}
          });
          
          if (response.ok) {
            document.getElementById('popupMessage').textContent = 'Action completed successfully!';
            document.getElementById('popupConfirm').style.display = 'none';
            document.getElementById('popupCancel').textContent = 'Close';
            document.getElementById('popupCancel').disabled = false;
            document.getElementById('popupLoading').classList.remove('show');
            
            if (currentAction === '/reboot') {
              setTimeout(() => closePopup(), 2000);
            }
          }
        } catch (error) {
          document.getElementById('popupMessage').textContent = 'Error: ' + error.message;
        } finally {
          document.getElementById('popupLoading').classList.remove('show');
        }
      });

      document.getElementById('popupCancel').addEventListener('click', closePopup);
      document.getElementById('popupOverlay').addEventListener('click', function(e) {
        if (e.target === this) closePopup();
      });

      document.addEventListener("DOMContentLoaded", () => {
        document.body.classList.add("fade-in");

        document.querySelectorAll("a").forEach(link => {
          const href = link.getAttribute("href");

          if (
            href &&
            !href.startsWith("http") &&
            !href.startsWith("#") &&
            !href.startsWith("javascript") &&
            !href.includes("?") &&
            !link.hasAttribute("download") &&
            link.target !== "_blank"
          ) {
            link.addEventListener("click", function(e) {
              e.preventDefault();
              document.body.classList.remove("fade-in");
              document.body.style.opacity = 0;
              setTimeout(() => {
                window.location.href = this.href;
              }, 300);
            });
          }
        });
      });
    </script>

    <div class='topnav'>
      <a href='/'>Home</a>
      )rawliteral"

      + wifi_button +

      R"rawliteral(
      <a href='/dir'>Files</a>
      <a href='/update'>OTA</a>
      <a href='/system'>System</a>
    </div>
  </body>
  )rawliteral";
}

void Handle_Home(AsyncWebServerRequest *request) {
  String page = HTML_Header();
  page += R"rawliteral(
    <style>
      .home_container {
        max-width: 800px;
        margin: 4em auto;
        background: rgba(38, 38, 38, 0.5);
        border-radius: 1.5em;
        backdrop-filter: blur(8px);
        box-shadow: 0 6px 20px rgba(0, 0, 0, 0.6);
        padding: 2.5em;
        text-align: center;
        animation: fadeIn 1.2s ease-out;
      }

      .home_container img {
        width: 100px;
        height: auto;
        margin: 1em 0;
        filter: drop-shadow(0 0 6px rgba(255,255,255,0.2));
        transition: transform 0.3s ease;
      }

      .home_container img:hover {
        transform: scale(1.1);
      }

      .home_container h1 {
        font-size: 2.5em;
        margin-bottom: 0.4em;
        color: #ffffff;
        text-shadow: 2px 2px 6px rgba(0, 0, 0, 0.7);
      }

      .home_container h2 {
        font-size: 1.3em;
        font-weight: 400;
        color: #dddddd;
        margin-bottom: 1em;
        text-shadow: 1px 1px 3px rgba(0, 0, 0, 0.6);
      }

      .home_container h3 {
        margin-top: 2em;
        font-size: 1.1em;
        color: #eeeeee;
        opacity: 0.8;
        letter-spacing: 1px;
      }

      @keyframes fadeIn {
        from { opacity: 0; transform: translateY(20px); }
        to { opacity: 1; transform: translateY(0); }
      }
    </style>

    <div class="home_container">
      <h1>Adi's Touchscreen Node</h1>
      <h2>Asynchronous WebServer</h2>
      <img src='/icon_)rawliteral" + String(chosen_icon) + R"rawliteral(' alt='icon'>
      <h3>Protein Protein Protein!!!</h3>
    </div>
  )rawliteral";

  request->send(200, "text/html", page);
}

void Handle_Wifi(AsyncWebServerRequest *request) {

  scan_now = true;
  scan_complete = true;
  last_scan = 0;

  String page = HTML_Header();

  page += R"rawliteral(
  <style>
    .wifi_box {
      background: rgba(38, 38, 38, 0.5);
      backdrop-filter: blur(5px);
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.5);
      border-radius: 1em;
      padding: 1.5em;
      margin: 3em auto;
      width: 300px;
      color: white;
      text-align: center;
      animation: fadeIn 1.2s ease-out;
    }

    .wifi_input {
      width: 100%;
      padding: 0.75em 1em;
      border-radius: 0.75em;
      border: none;
      background-color: rgba(255, 255, 255, 0.1);
      color: white;
      font-size: 1em;
      box-sizing: border-box;
    }

    .wifi_button {
      width: 100%;
      background-color: rgba(255, 255, 255, 0.1);
      color: white;
      border: none;
      border-radius: 0.75em;
      font-size: 1.1em;
      padding: 0.9em;
      cursor: pointer;
      transition: background 0.3s, transform 0.2s;
    }

    .wifi_button:hover {
      background-color: rgba(255, 255, 255, 0.3);
      transform: scale(1.02);
    }

    .wifi_network {
      display: flex;
      align-items: center;
      gap: 1em;
      margin: 1em 0;
      cursor: pointer;
      padding: 0.5em;
      border-radius: 0.75em;
      transition: background 0.3s, transform 0.2s;
    }
    .wifi_network:hover {
      background: rgba(255, 255, 255, 0.05);
      transform: scale(1.02);
    }
    
    .wifi_input::placeholder {
      color: rgba(255, 255, 255, 0.7);
    }

    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(20px); }
      to { opacity: 1; transform: translateY(0); }
    }
  </style>

  <div class='wifi_box'>
    <h2>Connect to Wi-Fi</h2>

    <div id='ssid_list'>
      <img src='/wifi_loading' width='140' height='105'><p>Scanning...</p>
    </div>

    <form action="/wifi_save" method="GET" style="margin-top: 2em;">
      <div style="margin-bottom: 1em;">
        <input class="wifi_input" type="text" name="ssid" placeholder="Enter SSID">
      </div>
      <div style="margin-bottom: 1.5em;">
        <input class="wifi_input" type="password" name="password" placeholder="Enter Password">
      </div>
      <button type="submit" class="wifi_button">Connect</button>
    </form>
  </div>

  <script>
    function fetchSSIDList() {
      fetch('/wifi_list')
        .then(response => response.text())
        .then(data => {
          document.getElementById('ssid_list').innerHTML = data;

          if (!data.includes('Scanning')) {
            clearInterval(pollInterval);
          }
        });
    }
    const pollInterval = setInterval(fetchSSIDList, 2000);
  </script>
  )rawliteral";

  request->send(200, "text/html", page);
}

void Handle_Wifi_List(AsyncWebServerRequest *request) {

  String page;

  if(scan_complete)
  {
    for(int i = 0; i < wifiSSIDCount; i++) 
    {
      if(wifiSSIDs[i].duplicate) continue;

      int rssi = wifiSSIDs[i].RSSI;
      String iconStrength;

      if (rssi >= -60)
        iconStrength = "full";
      else
        iconStrength = "half";

      bool isOpen = (wifiSSIDs[i].encryptionType == WIFI_AUTH_OPEN);
      String lockStatus = isOpen ? "unlocked" : "locked";
      String iconFile = "/wifi_" + lockStatus + "_" + iconStrength;

      String ssidName = wifiSSIDs[i].SSID;

      String safeSSID = ssidName;
      safeSSID.replace("'", "\\'");
      page += "<div class='wifi_network' onclick=\"document.getElementsByName('ssid')[0].value='" + safeSSID + "'\">";
      page += "<img src='" + iconFile + "' width='24' height='24'>";
      page += "<span>" + ssidName + "</span>";
      page += "</div>";
    }
  }
  else
  {
    page = "<img src='/wifi_loading' width='140' height='105'><p>Scanning...</p>";
  }

  request->send(200, "text/html", page);
}

void Handle_Wifi_Save(AsyncWebServerRequest *request) {

  _ssid = request->arg("ssid");
  _pass = request->arg("password");
    
  String page = HTML_Header();
  page += R"rawliteral(
  <style>
    .wifi_box {
      background: rgba(38, 38, 38, 0.5);
      backdrop-filter: blur(5px);
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.5);
      border-radius: 1em;
      padding: 1.5em;
      margin: 3em auto;
      width: fit-content;
      color: white;
      text-align: center;
    }

    .wifi_box p {
      font-size: 1.1em;
    }

    .ssid_name {
      color: #ccc;
      font-weight: normal;
    }
  </style>

  <div class="wifi_box">
    <h2>Wi-Fi Credentials Saved</h2>
    <p>SSID: <span class="ssid_name">)rawliteral" + String(_ssid) + R"rawliteral(</span></p>
    <p>You will be redirected in <span id="countdown">10</span> seconds.</p>
  </div>

  <script>
    let seconds = 10;
    const countdownEl = document.getElementById('countdown');

    const countdownInterval = setInterval(() => {
      seconds--;
      countdownEl.textContent = seconds;

      if (seconds <= 0) {
        clearInterval(countdownInterval);
        window.location.href = '/';
      }
    }, 1000);
  </script>
  )rawliteral";
  request->send(200, "text/html", page);
  connect_to_new_network = true;
}

void connectToNewWiFi() {
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WIFI_Connect(_ssid, _pass);
  connect_to_new_network = false;
}

int countFilesInDirectory(String path) {
  int count = 0;
  File dir = SD.open(path);
  if(dir) 
  {
    File file = dir.openNextFile();
    while(file) 
    {
      count++;
      file.close();
      file = dir.openNextFile();
    }
    dir.close();
  }
  return count;
}

int getFileTypePriority(String filename, String ftype) {
  if(ftype == "Dir") return 0;
  if(filename.endsWith(".mp4") || filename.endsWith(".avi") || filename.endsWith(".gif") || filename.endsWith(".mjpeg")) return 1;
  if(filename.endsWith(".jpg") || filename.endsWith(".png") || filename.endsWith(".bmp")) return 2;
  if(filename.endsWith(".mp3") || filename.endsWith(".wav") || filename.endsWith(".aac")) return 3;
  if(filename.endsWith(".txt")) return 4;
  return 5;
}

void SD_Directory(String path = "/") {
  numfiles = 0;
  File root = SD.open(path);
  if(root) 
  {
    root.rewindDirectory();
    File file = root.openNextFile();
    while(file && numfiles < MAX_FILES) 
    {
      const char* name = file.name();
      String filename = (name[0] == '/' ? String(name + 1) : String(name));
      
      int lastSlash = filename.lastIndexOf('/');
      if(lastSlash != -1) filename = filename.substring(lastSlash + 1);
      
      Filenames[numfiles].filename = filename;
      Filenames[numfiles].ftype = (file.isDirectory() ? "Dir" : "File");
      
      if(file.isDirectory()) {
        String fullPath = path;
        if(!fullPath.endsWith("/")) fullPath += "/";
        fullPath += filename;
        int fileCount = countFilesInDirectory(fullPath);
        Filenames[numfiles].fsize = String(fileCount) + " items";
      } else {
        Filenames[numfiles].fsize = ConvBinUnits(file.size(), 1);
      }
      
      file.close();
      file = root.openNextFile();
      numfiles++;
    }
    root.close();
  }
  
  for(int i = 0; i < numfiles - 1; i++) {
    for(int j = i + 1; j < numfiles; j++) {
      int priority_i = getFileTypePriority(Filenames[i].filename, Filenames[i].ftype);
      int priority_j = getFileTypePriority(Filenames[j].filename, Filenames[j].ftype);
      
      if(priority_i > priority_j) {
        fileinfo temp = Filenames[i];
        Filenames[i] = Filenames[j];
        Filenames[j] = temp;
      }
    }
  }
}

void Handle_SD_Dir(AsyncWebServerRequest *request) {
  String currentPath = "/";
  if(request->hasParam("path")) 
  {
    currentPath = request->getParam("path")->value();
    if(!currentPath.startsWith("/")) currentPath = "/" + currentPath;
  }
  
  String Fname1, Fname2;
  String icon1, icon2;
  String Fsize1, Fsize2;
  int index = 0;
  SD_Directory(currentPath);

  String page = HTML_Header();
  page += R"rawliteral(
  <style>
    .file_box {
      background: rgba(38, 38, 38, 0.5);
      backdrop-filter: blur(5px);
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.5);
      border-radius: 1em;
      padding: 2em;
      margin: 3em auto;
      width: fit-content;
      color: white;
      font-family: "Segoe UI", sans-serif;
      font-size: 1.05em;
      animation: fadeIn 1.2s ease-out;
    }

    .path_display {
      background: rgba(255, 255, 255, 0.1);
      padding: 0.75em 1.2em;
      border-radius: 0.75em;
      margin-bottom: 1.5em;
      font-family: monospace;
      text-align: center;
    }

    .file_controls {
      display: flex;
      justify-content: center;
      flex-wrap: wrap;
      gap: 1em;
      margin-bottom: 2em;
      animation: fadeIn 1.2s ease-out;
    }

    .file_controls a {
      background: rgba(255, 255, 255, 0.1);
      padding: 0.75em 1.2em;
      color: white;
      border-radius: 0.75em;
      text-decoration: none;
      font-weight: bold;
      transition: background 0.2s ease;
    }

    .file_controls a:hover {
      background-color: rgba(255, 255, 255, 0.3);
      transform: scale(1.02);
    }

    table {
      width: 100%;
      border-collapse: collapse;
      background-color: rgba(255, 255, 255, 0.05);
      border-radius: 0.75em;
      overflow: hidden;
      table-layout: fixed;
    }

    th, td {
      padding: 0.9em;
      text-align: left;
      border-bottom: 1px solid rgba(255,255,255,0.2);
      word-wrap: break-word;
      vertical-align: top;
    }

    .file_group {
      display: flex;
      flex-direction: column;
      gap: 0.6em;
      padding: 1.2em;
      transition: background 0.2s ease;
      border-radius: 0.75em;
    }

    .file_group:hover {
      background-color: rgba(255, 255, 255, 0.07);
      cursor: pointer;
    }

    .file_name {
      display: flex;
      align-items: center;
      gap: 0.8em;
      font-weight: 500;
      font-size: 1.1em;
    }

    .file_name img {
      width: 32px;
      height: 32px;
      object-fit: contain;
    }

    td.divider {
      width: 2px;
    }

    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(20px); }
      to { opacity: 1; transform: translateY(0); }
    }
  </style>

  <div class='file_box'>
    <h2>📁 File Manager</h2>
    <div class='path_display'>)rawliteral";
  
  page += currentPath;
  page += R"rawliteral(</div>
    <div class='file_controls'>)rawliteral";
  
  if(currentPath != "/") {
    page += "<a href=\"#\" onclick=\"goBack(); return false;\">Go Back</a>";
  }
 
  page += R"rawliteral(
      <a href='/upload'>Upload</a>
      <a href="#" onclick="showCreateFolder(); return false;">New Folder</a>
      <a href="#" onclick="toggleMode('open'); return false;">Open</a>
      <a href="#" onclick="toggleMode('download'); return false;">Download</a>
      <a href="#" onclick="toggleMode('rename'); return false;">Rename</a>
      <a href="#" onclick="toggleMode('move'); return false;">Move</a>
      <a href="#" onclick="toggleMode('delete'); return false;">Delete</a>
    </div>
  )rawliteral";

  if(numfiles > 0)
  {
    page += "<table>";

    while(index < numfiles) 
    {
      Fname1 = Filenames[index].filename;
      Fsize1 = Filenames[index].fsize;

      if(Filenames[index].ftype == "Dir")
      {
        icon1 = "/folder_icon";
      }
      else
      {
        if (Fname1.endsWith(".jpg") || Fname1.endsWith(".png") || Fname1.endsWith(".bmp")) icon1 = "/img_icon";
        else if (Fname1.endsWith(".mp4") || Fname1.endsWith(".avi") || Fname1.endsWith(".gif") || Fname1.endsWith(".mjpeg")) icon1 = "/video_icon";
        else if (Fname1.endsWith(".mp3") || Fname1.endsWith(".wav") || Fname1.endsWith(".aac")) icon1 = "/audio_icon";
        else if (Fname1.endsWith(".txt")) icon1 = "/txt_icon";
        else icon1 = "/file_icon";
      }

      if(index + 1 < numfiles)
      {
        Fname2 = Filenames[index + 1].filename;
        Fsize2 = Filenames[index + 1].fsize;

        if(Filenames[index + 1].ftype == "Dir")
        {
          icon2 = "/folder_icon";
        }
        else
        {
        if (Fname2.endsWith(".jpg") || Fname2.endsWith(".png") || Fname2.endsWith(".bmp")) icon2 = "/img_icon";
        else if (Fname2.endsWith(".mp4") || Fname2.endsWith(".avi") || Fname2.endsWith(".gif") || Fname2.endsWith(".mjpeg")) icon2 = "/video_icon";
        else if (Fname2.endsWith(".mp3") || Fname2.endsWith(".wav") || Fname2.endsWith(".aac")) icon2 = "/audio_icon";
        else if (Fname2.endsWith(".txt")) icon2 = "/txt_icon";
        else icon2 = "/file_icon";
        }
      }
      else
      {
        Fname2 = "";
        Fsize2 = "";
        icon2 = "";
      }

      page += "<tr>";
      page += "<td colspan='3'>";
      page += "<div class='file_group' data-filename='" + Fname1 +"' data-type='" + Filenames[index].ftype + "'>";
      page += "<div class='file_name'>";
      page += "<img src='" + icon1 + "'>";
      page += Fname1;
      page += "</div>";
      if(Filenames[index].ftype == "Dir") {
        page += "<div><strong>Items: </strong>" + Fsize1 + "</div>";
      } else {
        page += "<div><strong>Size: </strong>" + Fsize1 + "</div>";
      }
      page += "</div>";
      page += "</td>";
      page += "<td class='divider'></td>";

      if(index + 1 < numfiles) 
      {
        page += "<td colspan='3'>";
        page += "<div class='file_group' data-filename='" + Fname2 +"' data-type='" + Filenames[index + 1].ftype + "'>";
        page += "<div class='file_name'>";
        page += "<img src='" + icon2 + "'>";
        page += Fname2;
        page += "</div>";
        if(Filenames[index + 1].ftype == "Dir") {
          page += "<div><strong>Items: </strong>" + Fsize2 + "</div>";
        } else {
          page += "<div><strong>Size: </strong>" + Fsize2 + "</div>";
        }
        page += "</div>";
        page += "</td>";
        page += "</tr>";
      } 
      else
      {
        page += "<td colspan='3'></td>";
      }
      page += "</tr>";
      index += 2;
    }
    page += R"rawliteral(
      </table>
      </div>

      <div id="filePopup" class="popup_overlay">
        <div class="popup_box">
          <div id="filePopup_text"></div>
          <div class="popup_buttons">
            <button class="confirm_btn" onclick="startFileAction()">Yes</button>
            <button class="cancel_btn" onclick="closeFilePopup()">Cancel</button>
          </div>
        </div>
      </div>

      <script>
        let mode = '';
        let selectedFiles = [];
        const currentPath = ')rawliteral";
        page += currentPath;
        page += R"rawliteral(';

        function goBack() {
          const parts = currentPath.split('/').filter(p => p);
          parts.pop();
          const newPath = '/' + parts.join('/');
          window.location.href = '/dir?path=' + encodeURIComponent(newPath);
        }

        function toggleMode(selected) {
          mode = selected;
          selectedFiles = [];
          const header = document.querySelector('.file_box h2');
          const buttons = document.querySelectorAll('.file_group');
          const popup = document.getElementById('filePopup');

          buttons.forEach(el => {
            el.style.backgroundColor = '';
            el.removeEventListener('click', fileClickHandler);
          });
          popup.style.display = 'none';

          if (mode === 'open') {
            header.textContent = 'Click on a file or folder to open';
          } else if (mode === 'download') {
            header.textContent = 'Click on a file to download';
          } else if (mode === 'delete') {
            header.textContent = 'Select files/folders to delete (0 selected)';
          } else if (mode === 'rename') {
            header.textContent = 'Click on a file/folder to rename';
          } else if (mode === 'move') {
            header.textContent = 'Click on a file/folder to move';
          } else {
            header.textContent = '📁 File Manager';
            return;
          }

          buttons.forEach(el => el.addEventListener('click', fileClickHandler));
        }

        function fileClickHandler(e) {
          const filename = e.currentTarget.getAttribute('data-filename');
          const type = e.currentTarget.getAttribute('data-type');
          const popup = document.getElementById('filePopup');
          const popupText = document.getElementById('filePopup_text');
          const header = document.querySelector('.file_box h2');

          const fullPath = currentPath === '/' ? '/' + filename : currentPath + '/' + filename;

          if (mode === 'open') {
            if (type === 'Dir') {
              const newPath = currentPath === '/' ? '/' + filename : currentPath + '/' + filename;
              window.location.href = '/dir?path=' + encodeURIComponent(newPath);
            } else {
              const ext = filename.split('.').pop().toLowerCase();
              const images = ['jpg', 'jpeg', 'png', 'bmp', 'gif'];
              const videos = ['mp4', 'avi', 'webm', 'mjpeg'];
              const audio  = ['mp3', 'wav', 'aac'];
              const text   = ['txt'];
              if (images.includes(ext) || videos.includes(ext) || audio.includes(ext) || text.includes(ext)) {
                window.open(fullPath, '_blank');
              } else {
                alert("No preview available for this file type.");
              }
            }
            return;
          }

          if (mode === 'delete') {
            e.stopPropagation();
            const index = selectedFiles.indexOf(fullPath);
            if (index > -1) {
              selectedFiles.splice(index, 1);
              e.currentTarget.style.backgroundColor = '';
            } else {
              selectedFiles.push(fullPath);
              e.currentTarget.style.backgroundColor = 'rgba(255, 0, 0, 0.3)';
            }
            
            header.textContent = 'Select files/folders to delete (' + selectedFiles.length + ' selected)';
            
            if (selectedFiles.length > 0) {
              popupText.innerHTML = 'Delete ' + selectedFiles.length + ' item(s)?';
              popup.style.display = 'flex';
            } else {
              popup.style.display = 'none';
            }
            return;
          }

          if (mode === 'download') {
            popupText.innerHTML = 'Download "<b>' + filename + '</b>"?';
            selectedFiles = [fullPath];
            popup.style.display = 'flex';
          } else if (mode === 'rename') {
            popupText.innerHTML = 'Rename "<b>' + filename + '</b>": <br><input id="new_name" type="text" placeholder="New name" style="margin-top: 1em; width: 100%;">';
            selectedFiles = [fullPath];
            popup.style.display = 'flex';
          } else if (mode === 'move') {
            popupText.innerHTML = 'Move "<b>' + filename + '</b>" to:<br><input id="dest_path" type="text" placeholder="/destination/folder" value="' + currentPath + '" style="margin-top: 1em; width: 100%; padding: 0.5em;">';
            selectedFiles = [fullPath];
            popup.style.display = 'flex';
          }
        }

        async function startFileAction() {
          if (mode === 'createfolder') {
            const folderName = document.getElementById('folder_name').value.trim();
            if (!folderName) {
              alert("Enter folder name");
              return;
            }
            
            window.location.href = '/createfolder?path=' + encodeURIComponent(currentPath) + 
                                  '&name=' + encodeURIComponent(folderName);
            
            closeFilePopup();
            return;
          }

          if (selectedFiles.length === 0) return;

          if (mode === 'download') {
            const a = document.createElement('a');
            a.href = "/download?filename=" + encodeURIComponent(selectedFiles[0]);
            a.download = '';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            closeFilePopup();
          }

          else if (mode === 'delete') {
            closeFilePopup();
            
            let deleteCount = 0;
            for (let i = 0; i < selectedFiles.length; i++) {
              try {
                const response = await fetch('/delete?filename=' + encodeURIComponent(selectedFiles[i]) + '&path=' + encodeURIComponent(currentPath));
                if (response.ok) deleteCount++;
              } catch (err) {
                console.error('Delete failed:', err);
              }
            }
            
            window.location.href = '/dir?path=' + encodeURIComponent(currentPath);
          }

          else if (mode === 'rename') {
            const newName = document.getElementById('new_name').value;
            if (newName && newName.trim() !== "") {
              window.location.href = "/rename?old=" + encodeURIComponent(selectedFiles[0]) + "&new=" + encodeURIComponent(newName) + "&path=" + encodeURIComponent(currentPath);
            } else {
              alert("Enter a new filename");
              return;
            }
          }

          else if (mode === 'move') {
            const destPath = document.getElementById('dest_path').value;
            if (destPath && destPath.trim() !== "") {
              window.location.href = "/move?source=" + encodeURIComponent(selectedFiles[0]) + 
                                    "&destination=" + encodeURIComponent(destPath) + 
                                    "&path=" + encodeURIComponent(currentPath);
            } else {
              alert("Enter destination path");
              return;
            }
          }
        }

        function closeFilePopup() {
          const popup = document.getElementById('filePopup');
          popup.style.display = 'none';
          
          if (mode !== 'delete') {
            selectedFiles = [];
            const buttons = document.querySelectorAll('.file_group');
            buttons.forEach(el => el.style.backgroundColor = '');
          }
        }

        function showCreateFolder() {
          const popup = document.getElementById('filePopup');
          const popupText = document.getElementById('filePopup_text');
          
          popupText.innerHTML = 'Create new folder in current directory:<br><input id="folder_name" type="text" placeholder="Folder name" style="margin-top: 1em; width: 100%; padding: 0.5em;">';
          popup.style.display = 'flex';
          
          mode = 'createfolder';
        }
      </script>

      <style>
        .popup_overlay {
          position: fixed;
          top: 0; left: 0;
          width: 100%; height: 100%;
          background-color: rgba(0, 0, 0, 0.7);
          display: none;
          align-items: center;
          justify-content: center;
          z-index: 9999;
        }

        .popup_box {
          background: rgba(27, 27, 27, 1);
          padding: 2em;
          border-radius: 1em;
          color: white;
          text-align: center;
          width: 90%;
          max-width: 400px;
          box-shadow: 0 4px 16px rgba(0,0,0,0.5);
        }

        .popup_buttons {
          margin-top: 1.5em;
          display: flex;
          justify-content: space-around;
        }

        .popup_buttons button {
          padding: 0.6em 1.5em;
          border: none;
          border-radius: 0.5em;
          cursor: pointer;
          font-weight: bold;
          color: white;
          transition: background 0.2s ease;
        }

        .confirm_btn {
          background-color: rgba(91, 91, 91, 1);
        }

        .confirm_btn:hover {
          background-color: #00e676;
        }

        .cancel_btn {
          background-color: rgba(91, 91, 91, 1);
        }

        .cancel_btn:hover {
          background-color: #ef5350;
        }
      </style>
    )rawliteral";
  }
  else
  {
    if(!sd_available) page += "<h3>SD Card Not Mounted</h3>";
    else page += "<h3>No Files Found</h3>";
    page += "</div>";
  }
  request->send(200, "text/html", page);
}

void Handle_SD_File_Upload(AsyncWebServerRequest *request) {
  if(request->method() == HTTP_GET) 
  {
    String page = HTML_Header();
    page += R"rawliteral(
    <style>
      .file_box {
        background: rgba(38, 38, 38, 0.5);
        backdrop-filter: blur(5px);
        box-shadow: 0 4px 16px rgba(0, 0, 0, 0.5);
        border-radius: 1em;
        padding: 2em;
        margin: 3em auto;
        width: fit-content;
        color: white;
        font-family: "Segoe UI", sans-serif;
        font-size: 1.05em;
        animation: fadeIn 1.2s ease-out;
      }

      .upload_form {
        display: flex;
        flex-direction: column;
        gap: 1.5em;
        align-items: center;
      }

      .custom_file_input {
        position: relative;
        display: inline-block;
        overflow: hidden;
        border-radius: 0.75em;
        background: rgba(255, 255, 255, 0.1);
        cursor: pointer;
        font-weight: bold;
        padding: 0.75em 1.2em;
        color: white;
        transition: background 0.2s ease;
        width: 100%;
        text-align: center;
      }

      .custom_file_input:hover {
        background-color: rgba(255, 255, 255, 0.3);
        transform: scale(1.02);
      }

      .custom_file_input input[type="file"] {
        position: absolute;
        left: 0;
        top: 0;
        opacity: 0;
        cursor: pointer;
        width: 100%;
        height: 100%;
      }

      .filename_note {
        font-size: 0.9em;
        font-style: italic;
        color: #ccc;
      }

      .upload_button, .toggle_button {
        width: 100%;
        background: rgba(255, 255, 255, 0.1);
        padding: 0.75em 1.5em;
        color: white;
        border: none;
        border-radius: 0.75em;
        font-weight: bold;
        cursor: pointer;
        position: relative;
        overflow: hidden;
        transition: background 0.2s ease;
      }

      .upload_button:hover, .toggle_button:hover {
        background-color: rgba(255, 255, 255, 0.3);
        transform: scale(1.02);
      }

      .upload_button .progress_fill {
        background: linear-gradient(90deg, #00c6ff, #0072ff);
        position: absolute;
        left: 0;
        top: 0;
        height: 100%;
        width: 0%;
        z-index: 0;
        transition: width 0.2s ease;
      }

      .upload_button span {
        position: relative;
        z-index: 1;
      }

      @keyframes fadeIn {
        from { opacity: 0; transform: translateY(20px); }
        to { opacity: 1; transform: translateY(0); }
      }
    </style>

    <div class="file_box">
      <h2>Upload Files/Folders</h2>

      <div class="upload_form">
        <label class="custom_file_input">
          <span id="inputLabel">Choose Files</span>
          <input id="fileInput" type="file" multiple>
        </label>

        <button type="button" class="toggle_button" onclick="toggleUploadMode()">
          <span id="modeText">Switch to Folder Mode</span>
        </button>

        <div id="fileNameNote" class="filename_note">No files selected</div>

        <button type="button" class="upload_button" id="uploadBtn">
          <div class="progress_fill" id="progressFill"></div>
          <span id="uploadText">Upload</span>
        </button>
      </div>
    </div>
    <script>
      const fileInput = document.getElementById('fileInput');
      const fileNameNote = document.getElementById('fileNameNote');
      const uploadBtn = document.getElementById('uploadBtn');
      const uploadText = document.getElementById('uploadText');
      const progressFill = document.getElementById('progressFill');
      const inputLabel = document.getElementById('inputLabel');
      const modeText = document.getElementById('modeText');
      
      let folderMode = false;

      function toggleUploadMode() {
        folderMode = !folderMode;
        fileInput.value = '';
        
        if (folderMode) {
          fileInput.setAttribute('webkitdirectory', '');
          fileInput.setAttribute('directory', '');
          fileInput.removeAttribute('multiple');
          inputLabel.textContent = 'Choose Folder';
          modeText.textContent = 'Switch to File Mode';
          fileNameNote.textContent = 'No folder selected';
        } else {
          fileInput.removeAttribute('webkitdirectory');
          fileInput.removeAttribute('directory');
          fileInput.setAttribute('multiple', '');
          inputLabel.textContent = 'Choose Files';
          modeText.textContent = 'Switch to Folder Mode';
          fileNameNote.textContent = 'No files selected';
        }
      }

      fileInput.addEventListener('change', () => {
        const files = fileInput.files;
        if (files.length === 0) {
          fileNameNote.textContent = folderMode ? 'No folder selected' : 'No files selected';
        } else if (files.length === 1) {
          fileNameNote.textContent = `Selected: ${files[0].name}`;
        } else {
          fileNameNote.textContent = `Selected: ${files.length} files`;
        }
      });

      function getTotalSize(files) {
        let total = 0;
        for (let i = 0; i < files.length; i++) {
          total += files[i].size;
        }
        return total;
      }

      uploadBtn.addEventListener('click', async () => {
        const files = fileInput.files;
        if (files.length === 0) return alert('Please select files');

        uploadBtn.disabled = true;
        let totalUploaded = 0;

        for (let i = 0; i < files.length; i++) {
          const file = files[i];
          const formData = new FormData();
          formData.append('filename', file);
          formData.append('filepath', file.webkitRelativePath || file.name);

          try {
            await new Promise((resolve, reject) => {
              const xhr = new XMLHttpRequest();
              
              xhr.upload.onprogress = (e) => {
                if (e.lengthComputable) {
                  const filePercent = (e.loaded / e.total) * 100;
                  const overallPercent = ((totalUploaded + e.loaded) / getTotalSize(files)) * 100;
                  progressFill.style.width = overallPercent + '%';
                  uploadText.textContent = `${Math.round(overallPercent)}%`;
                }
              };

              xhr.onload = () => {
                if (xhr.status === 200) {
                  totalUploaded += file.size;
                  resolve();
                } else {
                  reject();
                }
              };

              xhr.onerror = () => reject();
              
              xhr.open('POST', '/upload', true);
              xhr.send(formData);
            });
          } catch (err) {
            console.error('Upload failed:', err);
          }
        }
        uploadText.textContent = 'Done!';
        setTimeout(() => {
          window.location.href = '/dir';
        }, 800);
      });
    </script>
    )rawliteral";
    request->send(200, "text/html", page);
    return;
  }
}

void on_SD_File_Upload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {

  static int start = 0;
  static int uploadtime = 0;
  static int uploadsize = 0;

  if(!index) 
  {
    String filepath = "/";

    if(request->hasArg("filepath"))
    {
      filepath += request->arg("filepath");
    }
    else
    {
      filepath += filename;
    }

    int lastSlash = filepath.lastIndexOf('/');
    if(lastSlash > 0)
    {
      String dirPath = filepath.substring(0, lastSlash);
      createDirectoryRecursive(dirPath);
    }

    if(request->_tempFile) request->_tempFile.close();

    request->_tempFile = SD.open(filepath, FILE_WRITE);
    if(!request->_tempFile)
    {
      DEBUG_PRINT("Failed to create file: " + filepath + "\n");
      return;
    }
    DEBUG_PRINT("Started upload: " + filepath + "\n");
    start = millis();
  }

  if(request->_tempFile && len) 
  {
    size_t written = request->_tempFile.write(data, len);
    if(written != len) {
      DEBUG_PRINT("Write error: expected " + String(len) + ", wrote " + String(written) + "\n");
    }
  }

  if(final && request->_tempFile) 
  {
    uploadsize = request->_tempFile.size();
    request->_tempFile.flush();
    request->_tempFile.close();
    uploadtime = millis() - start;
    float speed = (uploadsize / 1024.0) / (uploadtime / 1000.0);
    DEBUG_PRINT("Upload finished: " + String(uploadsize) + " bytes in " + String(uploadtime) + "ms (" + String(speed, 2) + " KB/s)\n");
    request->send(200);
  }
}

void createDirectoryRecursive(const String& path) {
  String currentPath = "";
  int start = 1;
  int end = path.indexOf('/', start);

  while(end != -1) 
  {
    currentPath += "/" + path.substring(start, end);
    if(!SD.exists(currentPath)) 
    {
      if(!SD.mkdir(currentPath)) DEBUG_PRINT("Failed to create directory: " + currentPath + "\n");
    }
    start = end + 1;
    end = path.indexOf('/', start);
  }

  if(start < path.length()) 
  {
    currentPath += "/" + path.substring(start);
    if(!SD.exists(currentPath)) SD.mkdir(currentPath);
  }
}

void Handle_SD_File_Download(AsyncWebServerRequest *request) {
  if(!request->hasParam("filename")) 
  {
    request->send(400, "text/plain", "Missing filename");
    DEBUG_PRINT("Download Handler failed, missing filename\n");
    return;
  }
  String filename = request->getParam("filename")->value();
  if(!SD.exists(filename)) 
  {
    request->send(404, "text/plain", "File not found");
    DEBUG_PRINT("Download Handler failed, file not found\n");
    return;
  }
  
  File file = SD.open(filename);
  if(file.isDirectory()) {
    file.close();
    request->send(400, "text/plain", "Cannot download folders directly");
    return;
  }
  file.close();
  
  String contentType = "application/octet-stream";
  if (filename.endsWith(".png")) contentType = "image/png";
  else if (filename.endsWith(".jpg") || filename.endsWith(".jpeg")) contentType = "image/jpeg";
  else if (filename.endsWith(".txt")) contentType = "text/plain";
  else if (filename.endsWith(".mp4")) contentType = "video/mp4";
  else if (filename.endsWith(".wav")) contentType = "audio/wav";
  request->send(SD, filename, contentType, true);
}

void deleteRecursive(String path) {
  File file = SD.open(path);
  if(!file) return;
  
  if(file.isDirectory()) 
  {
    file.rewindDirectory();
    File entry = file.openNextFile();
    while(entry) 
    {
      String entryPath = path;
      if(!entryPath.endsWith("/")) entryPath += "/";
      
      const char* name = entry.name();
      String entryName = (name[0] == '/' ? String(name + 1) : String(name));
      int lastSlash = entryName.lastIndexOf('/');
      if(lastSlash != -1) entryName = entryName.substring(lastSlash + 1);
      
      entryPath += entryName;
      
      if(entry.isDirectory()) 
      {
        entry.close();
        deleteRecursive(entryPath);
      } 
      else 
      {
        entry.close();
        SD.remove(entryPath);
      }
      entry = file.openNextFile();
    }
    file.close();
    SD.rmdir(path);
  } 
  else 
  {
    file.close();
    SD.remove(path);
  }
}

void Handle_SD_File_Delete(AsyncWebServerRequest *request) {
  if(!request->hasParam("filename")) 
  {
    request->send(400, "text/html", "Missing 'filename' parameter");
    return;
  }
  String filename = request->getParam("filename")->value();
  if(!filename.startsWith("/")) filename = "/" + filename;
  
  String currentPath = "/";
  if(request->hasParam("path")) 
  {
    currentPath = request->getParam("path")->value();
  } 
  else 
  {
    int lastSlash = filename.lastIndexOf('/');
    if(lastSlash > 0) currentPath = filename.substring(0, lastSlash);
  }
  
  if(SD.exists(filename)) 
  {
    deleteRecursive(filename);
    DEBUG_PRINT("Deleted: " + filename + "\n");
    request->send(200, "text/plain", "OK");
  } 
  else 
  {
    DEBUG_PRINT("Failed to delete, file not found\n");
    request->send(404, "text/plain", "Not found");
  }
}

void Handle_SD_File_Rename(AsyncWebServerRequest *request) {
  if(!request->hasParam("old") || !request->hasParam("new")) 
  {
    request->send(400, "text/html", "Missing 'old' or 'new' filename parameter");
    return;
  }
  String oldName = request->getParam("old")->value();
  String newName = request->getParam("new")->value();
  if(!oldName.startsWith("/")) oldName = "/" + oldName;
  
  String currentPath = "/";
  if(request->hasParam("path")) currentPath = request->getParam("path")->value();
  else 
  {
    int lastSlash = oldName.lastIndexOf('/');
    if(lastSlash > 0) currentPath = oldName.substring(0, lastSlash);
  }
  
  String newFullPath = currentPath;
  if(!newFullPath.endsWith("/")) newFullPath += "/";
  newFullPath += newName;
  
  if(oldName != newFullPath && oldName != "/" && newFullPath != "/") 
  {
    if(SD.exists(oldName)) 
    {
      if(!SD.exists(newFullPath)) 
      {
        if(SD.rename(oldName, newFullPath)) 
        {
          DEBUG_PRINT("Renamed from " + oldName + " to " + newFullPath + "\n");
        } 
        else 
        {
          DEBUG_PRINT("Failed to rename\n");
        }
      }
      else
      {
        DEBUG_PRINT("A file with the new name already exists\n");
      }
    }
    else
    {
      DEBUG_PRINT("Original file does not exist\n");
    }
  }
  request->redirect("/dir?path=" + currentPath);
}

void Handle_SD_File_Move(AsyncWebServerRequest *request) {
  if(!request->hasParam("source") || !request->hasParam("destination")) 
  {
    request->send(400, "text/html", "Missing parameters");
    return;
  }
  
  String sourcePath = request->getParam("source")->value();
  String destFolder = request->getParam("destination")->value();
  
  if(!sourcePath.startsWith("/")) sourcePath = "/" + sourcePath;
  if(!destFolder.startsWith("/")) destFolder = "/" + destFolder;
  if(!destFolder.endsWith("/")) destFolder += "/";
  
  String currentPath = "/";
  if(request->hasParam("path")) currentPath = request->getParam("path")->value();
  
  // Extract filename from source
  int lastSlash = sourcePath.lastIndexOf('/');
  String filename = sourcePath.substring(lastSlash + 1);
  
  String destPath = destFolder + filename;
  
  if(sourcePath != destPath && SD.exists(sourcePath)) 
  {
    if(!SD.exists(destPath)) 
    {
      if(SD.rename(sourcePath, destPath)) 
      {
        DEBUG_PRINT("Moved from " + sourcePath + " to " + destPath + "\n");
      } 
      else 
      {
        DEBUG_PRINT("Failed to move\n");
      }
    }
    else
    {
      DEBUG_PRINT("File already exists at destination\n");
    }
  }
  
  request->redirect("/dir?path=" + currentPath);
}

void Handle_Create_Folder(AsyncWebServerRequest *request) {
  if(!request->hasParam("path") || !request->hasParam("name")) 
  {
    request->send(400, "text/html", "Missing parameters");
    return;
  }
  
  String basePath = request->getParam("path")->value();
  String folderName = request->getParam("name")->value();
  
  if(!basePath.startsWith("/")) basePath = "/" + basePath;
  if(!basePath.endsWith("/")) basePath += "/";
  
  String fullPath = basePath + folderName;
  
  if(!SD.exists(fullPath)) 
  {
    if(SD.mkdir(fullPath)) 
    {
      DEBUG_PRINT("Created folder: " + fullPath + "\n");
    } 
    else 
    {
      DEBUG_PRINT("Failed to create folder\n");
    }
  }
  else
  {
    DEBUG_PRINT("Folder already exists\n");
  }
  
  request->redirect("/dir?path=" + basePath);
}

void Display_System_Info(AsyncWebServerRequest *request) {

  if(sd_available) SD_Directory();
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  String page = HTML_Header();

  page += R"rawliteral(
  <style>
    .info_card {
      background: rgba(38, 38, 38, 0.5);
      backdrop-filter: blur(6px);
      box-shadow: 0 4px 18px rgba(0, 0, 0, 0.6);
      border-radius: 1.2em;
      padding: 2em;
      margin: 2em auto;
      width: fit-content;
      color: white;
      font-family: "Segoe UI", sans-serif;
      font-size: 1em;
      animation: fadeIn 1.2s ease-out;
    }

    .info_card h3 {
      font-size: 1.5em;
      margin-bottom: 1em;
      text-align: center;
    }

    .info_card h4 {
      margin-top: 1.5em;
      margin-bottom: 0.8em;
    }

    .info_card form {
      margin-top: 2em;
    }

    .info_card table {
      border-collapse: collapse;
      width: 100%;
      margin-top: 1em;
    }

    .info_card th, .info_card td {
      padding: 0.5em 1em;
      border: 1px solid rgba(255, 255, 255, 0.2);
      text-align: left;
    }

    .info_card select, .info_card button {
      color: white;
      background: rgba(255, 255, 255, 0.1);
      padding: 0.75em 1.2em;
      text-decoration: none;
      font-size: 1.1em;
      transition: background 0.3s ease;
      border: none;
      border-radius: 0.75em;
      font-weight: bold;
      cursor: pointer;
      margin-top: 1em;
      margin-bottom: 1em;
    }

    .info_card select:hover, .info_card button:hover {
      background: rgba(255, 255, 255, 0.3);
      transform: scale(1.02);
    }

    .input-group {
      display: flex;
      align-items: center;
      gap: 0.75em;
      margin: 8px 0;
      background: rgba(255, 255, 255, 0.05);
      padding: 0.6em 0.9em;
      border-radius: 0.75em;
      flex-wrap: nowrap;
    }

    .input-group label {
      flex: 0 0 120px;
      text-align: left;
      font-weight: 600;
    }

    .input-group select {
      background: rgba(255, 255, 255, 0.1);
      color: white;
      border: 1px solid rgba(255, 255, 255, 0.2);
      border-radius: 0.5em;
      padding: 0.4em 0.6em;
      appearance: none;
      font-size: 0.95em;
      flex: 1;
    }

    .input-group select option {
      background-color: rgba(38, 38, 38, 0.95);
      color: #f0f0f0;
      padding: 0.5em;
    }

    .button-group {
      display: flex;
      flex-direction: column;
      gap: 0.8em;
      margin-top: 1.5em;
    }

    .button-group button {
      width: 100%;
    }

    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(20px); }
      to { opacity: 1; transform: translateY(0); }
    } 
  </style>
  )rawliteral";

  page += "<div class='info_card'>";
  page += "<h3>System Information</h3><table>";
  page += "<tr><th>Build Date</th><td>" + String(BUILD) + "</td></tr>";
  page += "<tr><th>Free PSRAM</th><td>" + ConvBinUnits(ESP.getFreePsram(), 1) + "</td></tr>";
  page += "</table>";

  page += "<h4>CPU Info</h4><table>";
  page += "<tr><th>CPU Cores</th><td>" + String(chip_info.cores) + "</td></tr>";
  page += "<tr><th>Chip Revision</th><td>" + String(chip_info.revision) + "</td></tr>";
  page += "<tr><th>Flash Type</th><td>" + String((chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Embedded" : "External") + "</td></tr>";
  page += "<tr><th>Flash Size</th><td>" + ConvBinUnits(ESP.getFlashChipSize(), 1) + "</td></tr>";
  page += "<tr><th>Free Heap</th><td>" + ConvBinUnits(ESP.getFreeHeap(), 1) + "</td></tr>";
  page += "</table>";

  page += "<h4>WiFi Info</h4><table>";
  page += "<tr><th>LAN IP</th><td>" + WiFi.localIP().toString() + "</td></tr>";
  page += "<tr><th>MAC Address</th><td>" + WiFi.BSSIDstr() + "</td></tr>";
  page += "<tr><th>SSID</th><td>" + WiFi.SSID() + "</td></tr>";
  page += "<tr><th>RSSI</th><td>" + String(WiFi.RSSI()) + " dB</td></tr>";
  page += "<tr><th>Channel</th><td>" + String(WiFi.channel()) + "</td></tr>";
  page += "<tr><th>Encryption</th><td>" + EncryptionType(WiFi.encryptionType(0)) + "</td></tr>";
  page += "</table>";

  page += "<h4>SD Card</h4><table>";
  page += "<tr><th>Total Space</th><td>" + ConvBinUnits(SD.totalBytes(), 1) + "</td></tr>";
  page += "<tr><th>Used Space</th><td>" + ConvBinUnits(SD.usedBytes(), 1) + "</td></tr>";
  page += "<tr><th>Free Space</th><td>" + ConvBinUnits(SD.totalBytes() - SD.usedBytes(), 1) + "</td></tr>";
  page += "</table>";

  page += "<div class='button-group'>";
  page += "<button onclick=\"showPopup('/reboot', 'Reboot Device', 'This will restart the device. Continue?')\">Reboot</button>";
  page += "<button onclick=\"showPopup('/rst_tch_cal', 'Reset Touch Calibration', 'Reset touch calibration to factory defaults?')\">Reset Touch Calibration</button>";
  page += "</div>";

  page += "</div>";

  request->send(200, "text/html", page);
}

void Handle_OTA(AsyncWebServerRequest *request) {
  String page = HTML_Header();
  page += R"rawliteral(
  <style>
    .file_box {
      background: rgba(38, 38, 38, 0.5);
      backdrop-filter: blur(5px);
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.5);
      border-radius: 1em;
      padding: 2em;
      margin: 3em auto;
      width: fit-content;
      color: white;
      font-family: "Segoe UI", sans-serif;
      font-size: 1.05em;
      animation: fadeIn 1.2s ease-out;
    }

    .upload_form {
      display: flex;
      flex-direction: column;
      gap: 1.5em;
      align-items: center;
    }

    .custom_file_input {
      position: relative;
      display: inline-block;
      overflow: hidden;
      border-radius: 0.75em;
      background: rgba(255, 255, 255, 0.1);
      cursor: pointer;
      font-weight: bold;
      padding: 0.75em 1.2em;
      color: white;
      transition: background 0.2s ease;
      width: 100%;
      text-align: center;
    }

    .custom_file_input:hover {
      background-color: rgba(255, 255, 255, 0.3);
      transform: scale(1.02);
    }

    .custom_file_input input[type="file"] {
      position: absolute;
      left: 0;
      top: 0;
      opacity: 0;
      cursor: pointer;
      width: 100%;
      height: 100%;
    }

    .filename_note {
      font-size: 0.9em;
      font-style: italic;
      color: #ccc;
    }

    .upload_button {
      width: 100%;
      background: rgba(255, 255, 255, 0.1);
      padding: 0.75em 1.5em;
      color: white;
      border: none;
      border-radius: 0.75em;
      font-weight: bold;
      cursor: pointer;
      position: relative;
      overflow: hidden;
      transition: background 0.2s ease;
    }

    .upload_button:hover {
      background-color: rgba(255, 255, 255, 0.3);
      transform: scale(1.02);
    }

    .upload_button .progress_fill {
      background: linear-gradient(90deg, #00c6ff, #0072ff);
      position: absolute;
      left: 0;
      top: 0;
      height: 100%;
      width: 0%;
      z-index: 0;
      transition: width 0.2s ease;
    }

    .upload_button span {
      position: relative;
      z-index: 1;
    }

    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(20px); }
      to { opacity: 1; transform: translateY(0); }
    }

  </style>

  <div class="file_box">
    <h2>OTA Firmware Update</h2>

    <form id="otaForm" class="upload_form" onsubmit="startOTA(event)">
      <label class="custom_file_input">
        Choose .bin File
        <input id="fileInput" type="file">
      </label>

      <div id="fileNameNote" class="filename_note">No file selected</div>

      <button type="submit" class="upload_button">
        <div class="progress_fill" id="progressFill"></div>
        <span id="uploadText">Start Update</span>
      </button>
    </form>
  </div>

  <script>
    const fileInput = document.getElementById('fileInput');
    const fileNameNote = document.getElementById('fileNameNote');
    const uploadText = document.getElementById('uploadText');
    const progressFill = document.getElementById('progressFill');

    fileInput.addEventListener('change', () => {
      const file = fileInput.files[0];
      fileNameNote.textContent = file ? `Selected: ${file.name}` : 'No file selected';
    });

    async function startOTA(e) {
      e.preventDefault();

      const file = fileInput.files[0];
      if (!file) return alert("Please select a .bin file first.");

      uploadText.textContent = "Starting...";

      const startRes = await fetch("/ota/start");
      if (!startRes.ok) {
        uploadText.textContent = "Start Failed!";
        return;
      }

      uploadText.textContent = "Uploading...";

      const formData = new FormData();
      formData.append("update", file, "firmware.bin");

      const xhr = new XMLHttpRequest();
      xhr.open("POST", "/ota/upload", true);

      xhr.upload.onprogress = (e) => {
        if (e.lengthComputable) {
          const percent = (e.loaded / e.total) * 100;
          progressFill.style.width = percent + "%";
        }
      };

      xhr.onload = () => {
        if (xhr.status === 200) {
          uploadText.textContent = "Upload Complete!";
          setTimeout(() => location.reload(), 3000);
        } else {
          uploadText.textContent = "Upload Failed!";
          progressFill.style.width = "0%";
        }
      };

      xhr.onerror = () => {
        uploadText.textContent = "Upload Error!";
      };

      xhr.send(formData);
    }
  </script>
  )rawliteral";

  request->send(200, "text/html", page);
}

void Handle_Page_Not_Found(AsyncWebServerRequest *request) {
  String page = HTML_Header();
  page += R"rawliteral(
    <style>
      .notfound {
        max-width: 800px;
        margin: 4em auto;
        background: rgba(38, 38, 38, 0.5);
        border-radius: 1.5em;
        backdrop-filter: blur(8px);
        box-shadow: 0 6px 20px rgba(0, 0, 0, 0.6);
        padding: 2.5em;
        text-align: center;
        animation: fadeIn 1.2s ease-out;
      }

      .notfound h1 {
        font-size: 2.5em;
        margin-bottom: 0.4em;
        color: #ffffff;
        text-shadow: 2px 2px 6px rgba(0, 0, 0, 0.7);
      }

      .notfound h2 {
        font-size: 1.3em;
        font-weight: 400;
        color: #dddddd;
        margin-bottom: 1em;
        text-shadow: 1px 1px 3px rgba(0, 0, 0, 0.6);
      }

      .notfound h3 {
        margin-top: 2em;
        font-size: 1.1em;
        color: #eeeeee;
        opacity: 0.8;
        letter-spacing: 1px;
      }

      @keyframes fadeIn {
        from { opacity: 0; transform: translateY(20px); }
        to { opacity: 1; transform: translateY(0); }
      }
    </style>

    <div class="notfound">
      <h1>Error 404</h1>
      <h2>Page Not Found</h2>
      <h3>The page you were looking for was not found, it may have been moved or is currently unavailable.</h3>
    </div>
  )rawliteral";
  request->send(200, "text/html", page);
}

String ConvBinUnits(uint64_t bytes, int resolution) {
  if(bytes < 1024) {
    return String((long long)bytes) + " B";
  }
  else if(bytes < 1024 * 1024) {
    return String((bytes / 1024.0), resolution) + " KB";
  }
  else if(bytes < (1024ULL * 1024 * 1024)) {
    return String((bytes / 1024.0 / 1024.0), resolution) + " MB";
  }
  else if(bytes < (1024ULL * 1024 * 1024 * 1024)) {
    return String((bytes / 1024.0 / 1024.0 / 1024.0), resolution) + " GB";
  }
  else return "";
}

String EncryptionType(wifi_auth_mode_t encryptionType) {
  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "OPEN";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2 PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA WPA2 PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2 ENTERPRISE";
    case (WIFI_AUTH_MAX):
      return "WPA2 MAX";
    default:
      return "";
  }
}

void WIFI_CopySSIDs(wifi_ssid_count_t n) {

  if (n == WIFI_SCAN_FAILED)
  {
    DEBUG_PRINT("scanNetworks returned: WIFI_SCAN_FAILED\n");
  }
  else if (n == WIFI_SCAN_RUNNING)
  {
    DEBUG_PRINT("scanNetworks returned: WIFI_SCAN_RUNNING!\n");
  }
  else if (n < 0)
  {
    DEBUG_PRINT("scanNetworks failed with unknown error code!\n");
  }
  else if (n == 0)
  {
    DEBUG_PRINT("No networks found\n");
  }
  else
  {
    DEBUG_PRINT("Scan complete. Found " + String(n) + " networks\n");
  }

  if(n > 0)
  {
    if(wifiSSIDs) delete[] wifiSSIDs;
    
    wifiSSIDs = new WiFiResult[n];
    wifiSSIDCount = n;

    for(wifi_ssid_count_t i = 0; i < n; i++)
    {
      wifiSSIDs[i].duplicate = false;

      WiFi.getNetworkInfo(i,
                          wifiSSIDs[i].SSID,
                          wifiSSIDs[i].encryptionType,
                          wifiSSIDs[i].RSSI,
                          wifiSSIDs[i].BSSID,
                          wifiSSIDs[i].channel);
    }

    for(int i = 0; i < n; i++)
    {
      for(int j = i + 1; j < n; j++)
      {
        if(wifiSSIDs[j].RSSI > wifiSSIDs[i].RSSI)
        {
          std::swap(wifiSSIDs[i], wifiSSIDs[j]);
        }
      }
    }

    String cssid;
    for(int i = 0; i < n; i++)
    {
      if(wifiSSIDs[i].duplicate == true) continue;

      cssid = wifiSSIDs[i].SSID;
      DEBUG_PRINT("STA " + String(i) + " - " + cssid + "\n");

      for(int j = i + 1; j < n; j++)
      {
        if(cssid == wifiSSIDs[j].SSID)
        {
          DEBUG_PRINT("DUP AP: " + String(wifiSSIDs[j].SSID) + "\n");
          wifiSSIDs[j].duplicate = true;
        }
      }
    }
  }
}

void WIFI_Connect(String ssid, String pass) {

  unsigned long started_at = millis();
  bool connecting = false; 

  if(ssid != "" && pass != "")//ssid and password provided
  {
    DEBUG_PRINT("SSID: " + ssid + " and pass: " + pass + "\n");
    connecting = true;
    WiFi.begin(ssid.c_str(), pass.c_str());
  }
  else//ssid and password NOT provided
  {
    DEBUG_PRINT("Searching for saved wifi networks\n");
    printLog("- Scanning for WiFi networks");

    scan_now = true;
    while(millis() - started_at < 10000) if(WIFI_Scan()) break;//start wifi scan

    if(wifiSSIDCount > 0)//if there is networks available
    {
      preferences.begin("wifi", false);

      for(int i = 0; i < NUM_SLOTS; i++)//check each slot to see if any of the available networks has been saved earlier
      {
        String ssid_key = "ssid_" + String(i);
        String pass_key = "pass_" + String(i);
        String stored_ssid = preferences.getString(ssid_key.c_str(), "");        
        String stored_pass = preferences.getString(pass_key.c_str(), "");

        DEBUG_PRINT("Saved network " + String(i) + " - SSID: " + stored_ssid + ", PSWD: " + stored_pass + "\n");

        for(int j = 0; j < wifiSSIDCount; j++)//go through the available networks
        {
          if(wifiSSIDs[j].duplicate) continue;
          String scanned_ssid = wifiSSIDs[j].SSID;

          if(stored_ssid == scanned_ssid)
          {
            DEBUG_PRINT("Connecting to saved network " + String(i) + " - SSID: " + stored_ssid + ", PSWD: " + stored_pass + "\n");
            printLog("- Connecting to " + stored_ssid);
            WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());
            connecting = true;
            break;
          }
        }
        if(connecting) break;
      }
      preferences.end();
    }
    else//no stations available to connect or scan failed
    {
      DEBUG_PRINT("No stations to connect\n");
    }
  } 

  if(connecting)//check if connection is successful
  {
    wifi_status = WiFi.waitForConnectResult();

    DEBUG_PRINT("Connection result: " + String(wifi_status) + "\n");

    if(wifi_status == WL_CONNECTED)
    {
      DEBUG_PRINT("WiFi connected, IP:" + WiFi.localIP().toString() + "\n");
      printLog("- WiFi connected, IP:" + WiFi.localIP().toString());
    } 
  }

  if(wifi_status == WL_CONNECTED && ssid != "" && pass != "")//connection was successful, save this network 
  {
    preferences.begin("wifi", false);

    int ssid_stored_at = -1;
    int emptySlot = -1;

    for(int i = 0; i < NUM_SLOTS; i++)//check each slot to see if the current wifi credentials has been saved
    {
      String key = "ssid_" + String(i);
      String stored_ssid = preferences.getString(key.c_str(), "");

      if(stored_ssid == "" && emptySlot == -1) emptySlot = i;//store the index of first empty slot

      DEBUG_PRINT("STORED SSID " + String(i) + " - " + stored_ssid + "\n");

      if(stored_ssid == ssid)
      {
        DEBUG_PRINT("Wifi already saved at " + String(i) +"\n");
        String pass_key = "pass_" + String(i);//update the password in case it has changed
        preferences.putString(pass_key.c_str(), pass);
        ssid_stored_at = i;
        break;
      }
    }

    if(ssid_stored_at == -1)//if not saved, then add the new wifi ssid and password to eeprom
    {
      if(emptySlot != -1)//there is an empty slot
      {
        String ssid_key = "ssid_" + String(emptySlot);
        String pass_key = "pass_" + String(emptySlot);

        preferences.putString(ssid_key.c_str(), ssid);
        preferences.putString(pass_key.c_str(), pass);

        DEBUG_PRINT("Wifi saved to empty slot at index " + String(emptySlot) + "\n");
      }
      else //if all slots are full then delete the first one and roll everything ahead
      {
        for (int i = 1; i < NUM_SLOTS; i++) {

          String ssid_key1 = "ssid_" + String(i - 1);
          String pass_key1 = "pass_" + String(i - 1);

          String ssid_key2 = "ssid_" + String(i);
          String pass_key2 = "pass_" + String(i);

          preferences.putString(ssid_key1.c_str(), preferences.getString(ssid_key2.c_str(), ""));
          preferences.putString(pass_key1.c_str(), preferences.getString(pass_key2.c_str(), ""));

          DEBUG_PRINT("NEW STORED SSID " + String(i - 1) + " - " + preferences.getString(ssid_key1.c_str(), "") + "\n");
        }

        //add the ssid and password into the last slot

        String ssid_key = "ssid_" + String(NUM_SLOTS - 1);
        String pass_key = "pass_" + String(NUM_SLOTS - 1);

        preferences.putString(ssid_key.c_str(), ssid);
        preferences.putString(pass_key.c_str(), pass);

        DEBUG_PRINT("NEW STORED SSID " + String(NUM_SLOTS - 1) + " - " + preferences.getString(ssid_key.c_str(), "") + "\n");

        DEBUG_PRINT("Wifi saved to last slot" + String(emptySlot) + "\n");
      }
    }
    preferences.end();
  }
}

bool WIFI_Scan() {

  wifi_ssid_count_t n = WiFi.scanComplete();

  if(last_scan == 0 || (millis() - last_scan >= 60000 && scan_now && n != WIFI_SCAN_RUNNING)) 
  {
    DEBUG_PRINT("About to scan.\n");

    last_scan = millis();
    int scanResult = WiFi.scanNetworks(true);

    DEBUG_PRINT("scanNetworks() returned: " + String(scanResult) + "\n");

    if(scanResult == WIFI_SCAN_FAILED) {
      DEBUG_PRINT("WIFI SCAN FAILED!\n");
    }
  }

  if(n >= 0)//Scan finished
  {
    printLog("- Found " + String(n) + " networks");
    scan_now = false;
    scan_complete = true;
    WIFI_CopySSIDs(n);
    WiFi.scanDelete();
    return true;
  }
  return false;
}

void Webserver_Init() {

  if(wifi_status != WL_CONNECTED) //captive server only if running on access point mode
  {
    DNS.setErrorReplyCode(DNSReplyCode::NoError);

    if(!DNS.start(53, "*", WiFi.softAPIP())) DEBUG_PRINT("Could not start Captive DNS Server!\n");
  }

  if(MDNS.begin(server_name)) {
    MDNS.addService("http", "tcp", 80);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    Handle_Home(request);
  });

  server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest * request) {
    Handle_Home(request);
  });

  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) {
    Handle_Home(request);
  });

  server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest *request) {
    Handle_Home(request);
  });

  if(wifi_status != WL_CONNECTED) //wifi options only if running on access point mode
  {
    server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest * request) {
      Handle_Wifi(request);
    });

    server.on("/wifi_list", HTTP_GET, [](AsyncWebServerRequest * request) {
      Handle_Wifi_List(request);
    });

    server.on("/wifi_save", HTTP_GET, [](AsyncWebServerRequest * request) {
      Handle_Wifi_Save(request);
    });
  }

  server.on("/dir", HTTP_GET, [](AsyncWebServerRequest * request) {
    Handle_SD_Dir(request);
  });

  server.on("/upload", HTTP_GET, Handle_SD_File_Upload);
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {}, on_SD_File_Upload);
  server.on("/download", HTTP_GET, Handle_SD_File_Download);
  server.on("/delete", HTTP_GET, Handle_SD_File_Delete);
  server.on("/rename", HTTP_GET, Handle_SD_File_Rename);
  server.on("/move", HTTP_GET, Handle_SD_File_Move);
  server.on("/createfolder", HTTP_GET, Handle_Create_Folder);

  server.on("/system", HTTP_GET, [](AsyncWebServerRequest * request) {
    Display_System_Info(request);
  });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Rebooting...");
    delay(500);
    ESP.restart();
  });

  if(sd_available) 
  {
    chosen_background = random(1, 3);
    chosen_icon = random(1, 3);

    if(wifi_status != WL_CONNECTED) //wifi options only if running on access point mode
    {
      server.serveStatic("/wifi_loading", SD, "/Assets/wifi_loading.gif").setCacheControl("max-age=86400");
      server.serveStatic("/wifi_locked_full", SD, "/Assets/wifi_locked_full.png").setCacheControl("max-age=86400");
      server.serveStatic("/wifi_locked_half", SD, "/Assets/wifi_locked_half.png").setCacheControl("max-age=86400");
      server.serveStatic("/wifi_locked_low", SD, "/Assets/wifi_locked_low.png").setCacheControl("max-age=86400");
      server.serveStatic("/wifi_unlocked_full", SD, "/Assets/wifi_unlocked_full.png").setCacheControl("max-age=86400");
      server.serveStatic("/wifi_unlocked_half", SD, "/Assets/wifi_unlocked_half.png").setCacheControl("max-age=86400");
      server.serveStatic("/wifi_unlocked_low", SD, "/Assets/wifi_unlocked_low.png").setCacheControl("max-age=86400");
    }

    server.serveStatic("/background_1", SD, "/Assets/background_1.jpg").setCacheControl("max-age=86400");
    server.serveStatic("/background_2", SD, "/Assets/background_2.jpg").setCacheControl("max-age=86400");
    server.serveStatic("/icon_1", SD, "/Assets/icon.jpg").setCacheControl("max-age=86400");
    server.serveStatic("/icon_2", SD, "/Assets/icon.gif").setCacheControl("max-age=86400");
    server.serveStatic("/img_icon", SD, "/Assets/img_icon.png").setCacheControl("max-age=86400");
    server.serveStatic("/video_icon", SD, "/Assets/video_icon.png").setCacheControl("max-age=86400");
    server.serveStatic("/txt_icon", SD, "/Assets/txt_icon.png").setCacheControl("max-age=86400");
    server.serveStatic("/file_icon", SD, "/Assets/file_icon.png").setCacheControl("max-age=86400");
    server.serveStatic("/folder_icon", SD, "/Assets/folder_icon.png").setCacheControl("max-age=86400");
    server.serveStatic("/audio_icon", SD, "/Assets/audio_icon.png").setCacheControl("max-age=86400");
    server.serveStatic("/", SD, "/");
  }

  server.on("/update", HTTP_GET, Handle_OTA);
  
  server.on("/ota/start", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      StreamString str;
      Update.printError(str);
      _update_error_str = str.c_str();
      _update_error_str.concat("\n");
      DEBUG_PRINT(_update_error_str.c_str());
    }
    request->send((Update.hasError()) ? 400 : 200, "text/plain", (Update.hasError()) ? _update_error_str.c_str() : "OK");
  });

  server.on("/ota/upload", HTTP_POST, 
    [](AsyncWebServerRequest *request){
      AsyncWebServerResponse *response = request->beginResponse(
        (Update.hasError()) ? 400 : 200, 
        "text/plain", 
        (Update.hasError()) ? _update_error_str.c_str() : "OK"
      );
      response->addHeader("Connection", "close");
      response->addHeader("Access-Control-Allow-Origin", "*");
      request->send(response);
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
      if(!index) {
        _current_progress_size = 0;
        DEBUG_PRINT("OTA upload started: " + filename + "\n");
      }
      if(len) {
        size_t written = Update.write(data, len);
        if(written != len) {
          DEBUG_PRINT("Write failed. Written " + String(written) + " of " + String(len) + "\n");
          return;
        }
        _current_progress_size += len;
      }
      if(final) {
        DEBUG_PRINT("OTA upload finished. Total size: " + String(_current_progress_size) + " bytes\n");
        if(!Update.end(true)) {
          StreamString str;
          Update.printError(str);
          _update_error_str = str.c_str();
          _update_error_str.concat("\n");
          DEBUG_PRINT(_update_error_str.c_str());
        } else {
          DEBUG_PRINT("Update complete. Rebooting...\n");
          ESP.restart();
        }
      }
    }
  );

  server.onNotFound(Handle_Page_Not_Found);
  
  server.begin();
  
  DEBUG_PRINTLN("Webserver started");
}

//========================================================================================================================================
//======================= Loops, Buttons and EEPROM Functions ========================
//========================================================================================================================================

void get_eeprom() {

  preferences.begin("variables", false);
  preferences.end();
}

void update_eeprom() {

  preferences.begin("variables", false);
  preferences.end();
}

void Matrix_Handler() {

}

void printLog(String log) {
  tft.drawString(log, 5, 8 + (10 * log_counter));
  log_counter++;
}

void setup() {

  #if SERIAL_DEBUG != 0
    Serial.begin(115200);
  #endif

  pinMode(DevBTN, INPUT);

  get_eeprom();

  DEBUG_PRINTLN("Build:" + String(BUILD));

  tft.init();
  tft.setRotation(0);
  tft.setTextSize(1);
  tft.fillScreen(TFT_BLACK);
  printLog("- Build:" + String(BUILD));
  printLog("- Hold button to enter dev mode");
  delay(1000);

  if(digitalRead(DevBTN))
  {
    DEBUG_PRINTLN("Button Held");
    printLog("- Button Held");
    webserver_mode = true;
  }

  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if(!SD.begin(SD_CS, sdSPI)) 
  {
    DEBUG_PRINTLN("SD Card Mount Failed");
    printLog("- SD Card Mount Failed!");
    sd_available = false;
  }
  else
  {
    DEBUG_PRINTLN("SD Card Mount Successful");
    printLog("- SD Card Mount Successful");
  }

  /*DEBUG_PRINTLN("Attempting WiFi Connection..");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WIFI_Connect("", "");
  if(wifi_status != WL_CONNECTED)
  {
    WiFi.disconnect(true, true);
    DEBUG_PRINT("Could not connect to wifi..\n");
    printLog("- Could not connect to wifi!");

    webserver_mode = true;

    if(wifi_status != WL_CONNECTED)
    {
      printLog("- Starting AP..");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_AP);
      delay(100);
      printLog("- Starting access point..");
      DEBUG_PRINT("Configuring access point: " + String(wifi_ap_ssid) + " with password: " + String(wifi_ap_pass) + "\n");
      WiFi.softAP(wifi_ap_ssid, wifi_ap_pass);
      delay(500);
      printLog("- AP Started:" + WiFi.softAPIP().toString());
      DEBUG_PRINT("AP Started, IP:" + WiFi.softAPIP().toString() + "\n");
    }
  }*/

  if(webserver_mode)
  {
    Webserver_Init();
    printLog("- Webserver Started");
  }
  else
  {
    Wire.end();
    if(!Wire.begin(I2C_SDA, I2C_SCL))
    {
      DEBUG_PRINTLN("I2C Init Failed!");
      printLog("- I2C Init Failed!");
    }
    else
    {
      DEBUG_PRINTLN("I2C Init Successful");
      printLog("- I2C Init Successful");
    }
    
    if(!ExtraIO.begin(SX1509_ADDRESS))
    {
      DEBUG_PRINTLN("SX1509 Init Failed!");
      printLog("- SX1509 Init Failed!");
    }
    else
    {
      DEBUG_PRINTLN("SX1509 Init Successful");
      printLog("- SX1509 Init Successful");
    }

    if(!LIDAR.begin())
    {
      DEBUG_PRINTLN("VL53L0X Init Failed!");
      printLog("- VL53L0X Init Failed!");
    }
    else
    {
      DEBUG_PRINTLN("VL53L0X Init Successful");
      printLog("- VL53L0X Init Successful");
    }

    if(!AHT.begin()) 
    {
      DEBUG_PRINTLN("AHT Init Failed!");
      printLog("- AHT Init Failed!");
    }
    else
    {
      DEBUG_PRINTLN("AHT Init Successful");
      printLog("- AHT Init Successful");
    }

    pixels.begin();
    pixels.clear();
    for(int i = 0; i < NUMPIXELS; i++) 
    {
      pixels.setPixelColor(i, pixels.Color(100, 0, 0));
      pixels.show();
      delay(100);
    }

    Serial.printf("Heap free:  %u bytes\n", ESP.getFreeHeap());
    return;
  }
}

void loop() {
  if(wifi_status != WL_CONNECTED) DNS.processNextRequest();
  if(scan_now) WIFI_Scan();
  if(connect_to_new_network) connectToNewWiFi();
}
