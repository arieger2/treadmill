/*
 * TrainingPeaks Workout Executor - Clean Implementation
 * 
 * Features:
 * - Web interface for workout management
 * - TrainingPeaks API integration
 * - Real-time workout execution
 * - Clean, simple code structure
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

// WiFi Configuration
const char* ssid = "chicago";
const char* password = "Neukirch20211!";

// Web Server
WebServer server(80);

// TrainingPeaks Configuration
const char* tp_api_base = "https://api.trainingpeaks.com";
String tp_username = "arieger";
String tp_password = "LisgumuM67";
String session_token = "";
bool authenticated = false;

// Workout Data Structures
struct WorkoutStep {
  String name;
  int duration;
  String intensity_type;
  float target_min;
  float target_max;
  int zone;
  String step_type;
};

struct Workout {
  String id;
  String name;
  String sport;
  int total_duration;
  int step_count;
  WorkoutStep steps[20];
};

// Global State
Workout workouts[10];
int workout_count = 0;
int selected_workout = 0;
int current_step = 0;
unsigned long step_start_time = 0;
unsigned long workout_start_time = 0;
bool workout_active = false;
bool workout_paused = false;
int elapsed_time = 0;
int step_elapsed_time = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting TrainingPeaks Workout Executor...");

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed");
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  // Load saved data
  loadUserConfig();
  loadCredentials();
  loadWorkoutsFromStorage();

  // Setup web server
  setupWebServer();
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
  
  if (workout_active && !workout_paused) {
    updateWorkoutExecution();
  }
  
  delay(50);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/workouts", handleWorkouts);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/auth", HTTP_POST, handleAuth);
  server.on("/api/import", HTTP_POST, handleImport);
  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/pause", HTTP_POST, handlePause);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.onNotFound([]() { server.send(404, "text/plain", "Not Found"); });
}

void handleRoot() {
  String page = "<!DOCTYPE html><html><head>";
  page += "<title>TrainingPeaks Workout Executor</title>";
  page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  page += "<style>";
  page += "body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#f4f4f4}";
  page += ".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.1)}";
  page += ".header{text-align:center;margin-bottom:30px;color:#333}";
  page += ".card{background:#f9f9f9;padding:15px;margin:15px 0;border-radius:5px;border-left:4px solid #007bff}";
  page += ".status{padding:10px;border-radius:5px;margin:10px 0;text-align:center;font-weight:bold}";
  page += ".status.ok{background:#d4edda;color:#155724}";
  page += ".status.error{background:#f8d7da;color:#721c24}";
  page += ".form-group{margin:15px 0}";
  page += ".form-group label{display:block;margin-bottom:5px;font-weight:bold}";
  page += ".form-group input{width:100%;padding:10px;border:1px solid #ddd;border-radius:3px;box-sizing:border-box}";
  page += ".btn{padding:10px 15px;margin:5px;border:none;border-radius:3px;cursor:pointer;font-size:14px}";
  page += ".btn-primary{background:#007bff;color:white}";
  page += ".btn-success{background:#28a745;color:white}";
  page += ".btn-warning{background:#ffc107;color:#212529}";
  page += ".btn-danger{background:#dc3545;color:white}";
  page += ".btn:disabled{opacity:0.5;cursor:not-allowed}";
  page += ".workout-list{margin:20px 0}";
  page += ".workout-item{padding:15px;margin:10px 0;border:1px solid #ddd;border-radius:5px;cursor:pointer;background:white}";
  page += ".workout-item:hover{background:#f0f0f0}";
  page += ".workout-item.selected{border-color:#007bff;background:#e7f3ff}";
  page += ".progress{width:100%;height:20px;background:#e9ecef;border-radius:10px;margin:10px 0}";
  page += ".progress-bar{height:100%;background:#007bff;border-radius:10px;transition:width 0.3s}";
  page += ".hidden{display:none}";
  page += "</style></head><body>";
  
  page += "<div class='container'>";
  page += "<div class='header'><h1>TrainingPeaks Workout Executor</h1></div>";
  page += "<div id='status' class='status'>Loading...</div>";
  
  // Configuration Section
  page += "<div class='card'>";
  page += "<h3>Configuration</h3>";
  page += "<div class='form-group'>";
  page += "<label>Username:</label>";
  page += "<input type='text' id='username' placeholder='TrainingPeaks username'>";
  page += "</div>";
  page += "<div class='form-group'>";
  page += "<label>Password:</label>";
  page += "<input type='password' id='password' placeholder='TrainingPeaks password'>";
  page += "</div>";
  page += "<button class='btn btn-primary' onclick='saveConfig()'>Save</button>";
  page += "<button class='btn btn-success' onclick='testAuth()'>Test</button>";
  page += "<button class='btn btn-warning' onclick='importWorkouts()'>Import</button>";
  page += "</div>";
  
  // Workouts Section
  page += "<div class='card'>";
  page += "<h3>Workouts</h3>";
  page += "<div id='workouts'></div>";
  page += "<div style='text-align:center;margin:20px 0'>";
  page += "<button class='btn btn-success' id='startBtn' onclick='startWorkout()' disabled>Start</button>";
  page += "<button class='btn btn-warning' id='pauseBtn' onclick='pauseWorkout()' disabled>Pause</button>";
  page += "<button class='btn btn-danger' id='stopBtn' onclick='stopWorkout()' disabled>Stop</button>";
  page += "</div>";
  page += "</div>";
  
  // Progress Section
  page += "<div class='card hidden' id='progressCard'>";
  page += "<h3 id='progressTitle'>Workout Progress</h3>";
  page += "<div class='progress'><div id='progressBar' class='progress-bar'></div></div>";
  page += "<div id='progressInfo'></div>";
  page += "</div>";
  
  page += "</div>";
  
  // JavaScript
  page += "<script>";
  page += "let selectedWorkout=-1,workouts=[],status={};";
  
  // Initialize
  page += "document.addEventListener('DOMContentLoaded',function(){";
  page += "loadStatus();loadWorkouts();setInterval(updateAll,2000);";
  page += "});";
  
  // Load functions
  page += "async function loadStatus(){";
  page += "try{";
  page += "const r=await fetch('/api/status');";
  page += "status=await r.json();";
  page += "updateUI();";
  page += "}catch(e){console.error(e);}";
  page += "}";
  
  page += "async function loadWorkouts(){";
  page += "try{";
  page += "const r=await fetch('/api/workouts');";
  page += "const data=await r.json();";
  page += "workouts=data.workouts||[];";
  page += "renderWorkouts();";
  page += "}catch(e){console.error(e);}";
  page += "}";
  
  // UI Updates
  page += "function updateUI(){";
  page += "const s=document.getElementById('status');";
  page += "const ok=status.authenticated&&status.wifi_connected;";
  page += "s.className=ok?'status ok':'status error';";
  page += "s.textContent=ok?'Connected':'Not Connected';";
  page += "document.getElementById('username').value=status.username||'';";
  page += "updateProgress();";
  page += "updateButtons();";
  page += "}";
  
  page += "function renderWorkouts(){";
  page += "const c=document.getElementById('workouts');";
  page += "if(workouts.length===0){";
  page += "c.innerHTML='<p>No workouts. Click Import to load from TrainingPeaks.</p>';";
  page += "return;";
  page += "}";
  page += "let html='';";
  page += "workouts.forEach((w,i)=>{";
  page += "const sel=i===selectedWorkout?' selected':'';";
  page += "html+='<div class=\"workout-item'+sel+'\" onclick=\"selectWorkout('+i+')\">';";
  page += "html+='<strong>'+w.name+'</strong><br>';";
  page += "html+='<small>'+Math.floor(w.total_duration/60)+' min | '+w.sport+'</small>';";
  page += "html+='</div>';";
  page += "});";
  page += "c.innerHTML=html;";
  page += "}";
  
  page += "function selectWorkout(i){";
  page += "selectedWorkout=i;";
  page += "renderWorkouts();";
  page += "updateButtons();";
  page += "}";
  
  page += "function updateButtons(){";
  page += "document.getElementById('startBtn').disabled=selectedWorkout===-1||status.workout_active;";
  page += "document.getElementById('pauseBtn').disabled=!status.workout_active;";
  page += "document.getElementById('stopBtn').disabled=!status.workout_active;";
  page += "document.getElementById('pauseBtn').textContent=status.workout_paused?'Resume':'Pause';";
  page += "}";
  
  page += "function updateProgress(){";
  page += "const card=document.getElementById('progressCard');";
  page += "if(status.workout_active){";
  page += "card.classList.remove('hidden');";
  page += "document.getElementById('progressTitle').textContent=status.current_workout_name||'Workout';";
  page += "const pct=status.total_duration>0?(status.elapsed_time/status.total_duration)*100:0;";
  page += "document.getElementById('progressBar').style.width=pct+'%';";
  page += "const info='Step: '+(status.current_step+1)+'/'+(status.total_steps||0)+'<br>';";
  page += "document.getElementById('progressInfo').innerHTML=info+'Time: '+formatTime(status.elapsed_time||0);";
  page += "}else{";
  page += "card.classList.add('hidden');";
  page += "}";
  page += "}";
  
  page += "function formatTime(s){";
  page += "const m=Math.floor(s/60);";
  page += "const sec=s%60;";
  page += "return m+':'+(sec<10?'0':'')+sec;";
  page += "}";
  
  // API Functions
  page += "async function saveConfig(){";
  page += "const u=document.getElementById('username').value;";
  page += "const p=document.getElementById('password').value;";
  page += "if(!u||!p){alert('Enter username and password');return;}";
  page += "try{";
  page += "const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({username:u,password:p})});";
  page += "alert(r.ok?'Saved':'Failed');";
  page += "if(r.ok)loadStatus();";
  page += "}catch(e){alert('Error');}";
  page += "}";
  
  page += "async function testAuth(){";
  page += "try{";
  page += "const r=await fetch('/api/auth',{method:'POST'});";
  page += "const result=await r.json();";
  page += "alert(result.success?'Success':'Failed: '+(result.error||'Unknown'));";
  page += "}catch(e){alert('Error');}";
  page += "}";
  
  page += "async function importWorkouts(){";
  page += "try{";
  page += "const r=await fetch('/api/import',{method:'POST'});";
  page += "const result=await r.json();";
  page += "alert(result.success?'Imported '+result.count+' workouts':'Failed: '+(result.error||'Unknown'));";
  page += "if(result.success){loadWorkouts();loadStatus();}";
  page += "}catch(e){alert('Error');}";
  page += "}";
  
  page += "async function startWorkout(){";
  page += "if(selectedWorkout===-1)return;";
  page += "try{";
  page += "const r=await fetch('/api/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({index:selectedWorkout})});";
  page += "if(r.ok)loadStatus();";
  page += "}catch(e){console.error(e);}";
  page += "}";
  
  page += "async function pauseWorkout(){";
  page += "try{";
  page += "await fetch('/api/pause',{method:'POST'});";
  page += "loadStatus();";
  page += "}catch(e){console.error(e);}";
  page += "}";
  
  page += "async function stopWorkout(){";
  page += "try{";
  page += "await fetch('/api/stop',{method:'POST'});";
  page += "loadStatus();";
  page += "}catch(e){console.error(e);}";
  page += "}";
  
  page += "function updateAll(){loadStatus();}";
  
  page += "</script></body></html>";
  
  server.send(200, "text/html", page);
}

void handleStatus() {
  DynamicJsonDocument doc(1024);
  doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  doc["authenticated"] = authenticated;
  doc["username"] = tp_username;
  doc["workout_count"] = workout_count;
  doc["workout_active"] = workout_active;
  doc["workout_paused"] = workout_paused;
  doc["current_step"] = current_step;
  doc["elapsed_time"] = elapsed_time;
  
  if (workout_active && workout_count > 0) {
    doc["current_workout_name"] = workouts[selected_workout].name;
    doc["total_steps"] = workouts[selected_workout].step_count;
    doc["total_duration"] = workouts[selected_workout].total_duration;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWorkouts() {
  DynamicJsonDocument doc(4096);
  JsonArray workoutsArray = doc.createNestedArray("workouts");
  
  for (int i = 0; i < workout_count; i++) {
    JsonObject workout = workoutsArray.createNestedObject();
    workout["name"] = workouts[i].name;
    workout["sport"] = workouts[i].sport;
    workout["total_duration"] = workouts[i].total_duration;
    workout["step_count"] = workouts[i].step_count;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleConfig() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No data\"}");
    return;
  }
  
  DynamicJsonDocument doc(512);
  deserializeJson(doc, server.arg("plain"));
  
  if (doc.containsKey("username")) tp_username = doc["username"].as<String>();
  if (doc.containsKey("password")) tp_password = doc["password"].as<String>();
  
  saveUserConfig();
  server.send(200, "application/json", "{\"success\":true}");
}

void handleAuth() {
  bool success = authenticateWithTrainingPeaks();
  
  DynamicJsonDocument doc(256);
  doc["success"] = success;
  if (!success) doc["error"] = "Authentication failed";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleImport() {
  if (!authenticated && !authenticateWithTrainingPeaks()) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Not authenticated\"}");
    return;
  }
  
  HTTPClient http;
  http.begin(String(tp_api_base) + "/v1/workouts");
  http.addHeader("Authorization", "Bearer " + session_token);
  
  int code = http.GET();
  if (code == 200) {
    parseWorkouts(http.getString());
    saveWorkoutsToStorage();
    
    DynamicJsonDocument doc(256);
    doc["success"] = true;
    doc["count"] = workout_count;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  } else {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"HTTP error\"}");
  }
  
  http.end();
}

void handleStart() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No data\"}");
    return;
  }
  
  DynamicJsonDocument doc(256);
  deserializeJson(doc, server.arg("plain"));
  
  if (doc.containsKey("index")) {
    int index = doc["index"];
    if (index >= 0 && index < workout_count) {
      selected_workout = index;
      startWorkout();
      server.send(200, "application/json", "{\"success\":true}");
      return;
    }
  }
  
  server.send(400, "application/json", "{\"error\":\"Invalid index\"}");
}

void handlePause() {
  if (workout_active) {
    workout_paused = !workout_paused;
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"No active workout\"}");
  }
}

void handleStop() {
  if (workout_active) {
    stopWorkout();
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"No active workout\"}");
  }
}

void startWorkout() {
  if (workout_count > 0) {
    workout_active = true;
    workout_paused = false;
    current_step = 0;
    workout_start_time = millis();
    step_start_time = millis();
    Serial.println("Workout started: " + workouts[selected_workout].name);
  }
}

void stopWorkout() {
  workout_active = false;
  workout_paused = false;
  current_step = 0;
  Serial.println("Workout stopped");
}

void updateWorkoutExecution() {
  unsigned long now = millis();
  elapsed_time = (now - workout_start_time) / 1000;
  step_elapsed_time = (now - step_start_time) / 1000;
  
  if (current_step < workouts[selected_workout].step_count) {
    if (step_elapsed_time >= workouts[selected_workout].steps[current_step].duration) {
      current_step++;
      step_start_time = now;
      
      if (current_step >= workouts[selected_workout].step_count) {
        workout_active = false;
        Serial.println("Workout completed");
      } else {
        Serial.println("Step " + String(current_step + 1) + ": " + workouts[selected_workout].steps[current_step].name);
      }
    }
  }
}

bool authenticateWithTrainingPeaks() {
  HTTPClient http;
  http.begin(String(tp_api_base) + "/v1/auth/login");
  http.addHeader("Content-Type", "application/json");
  
  DynamicJsonDocument loginDoc(512);
  loginDoc["username"] = tp_username;
  loginDoc["password"] = tp_password;
  
  String payload;
  serializeJson(loginDoc, payload);
  
  int code = http.POST(payload);
  Serial.printf("http payload: %s\n", payload.c_str());
  Serial.printf("http Code: %d\n", code);
  bool success = false;
  
  if (code == 200) {
    String response = http.getString();
    DynamicJsonDocument responseDoc(1024);
    DeserializationError err = deserializeJson(responseDoc, response);
    if (err) {
      Serial.println("Failed to parse JSON response");
      return false;
    }
    
    if (responseDoc.containsKey("access_token")) {
      session_token = responseDoc["access_token"].as<String>();
      authenticated = true;
      success = true;
      saveCredentials();
    }
  }
  
  http.end();
  return success;
}

void parseWorkouts(String json) {
  DynamicJsonDocument doc(8192);
  deserializeJson(doc, json);
  
  workout_count = 0;
  JsonArray workoutsArray = doc["workouts"].as<JsonArray>();
  
  for (JsonObject workoutObj : workoutsArray) {
    if (workout_count >= 10) break;
    
    Workout& workout = workouts[workout_count];
    workout.id = workoutObj["id"].as<String>();
    workout.name = workoutObj["name"].as<String>();
    workout.sport = workoutObj["sport"].as<String>();
    workout.total_duration = workoutObj["totalDuration"];
    workout.step_count = 0;
    
    JsonArray stepsArray = workoutObj["steps"].as<JsonArray>();
    for (JsonObject stepObj : stepsArray) {
      if (workout.step_count >= 20) break;
      
      WorkoutStep& step = workout.steps[workout.step_count];
      step.name = stepObj["name"].as<String>();
      step.duration = stepObj["duration"];
      step.intensity_type = stepObj["intensityType"].as<String>();
      step.target_min = stepObj["targetMin"];
      step.target_max = stepObj["targetMax"];
      step.zone = stepObj["zone"];
      step.step_type = stepObj["stepType"].as<String>();
      
      workout.step_count++;
    }
    
    workout_count++;
  }
  
  Serial.println("Parsed " + String(workout_count) + " workouts");
}

void saveUserConfig() {
  File file = SPIFFS.open("/config.json", "w");
  if (file) {
    DynamicJsonDocument doc(512);
    doc["username"] = tp_username;
    doc["password"] = tp_password;
    serializeJson(doc, file);
    file.close();
  }
}

void loadUserConfig() {
  File file = SPIFFS.open("/config.json", "r");
  if (file) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, file);
    if (doc.containsKey("username")) tp_username = doc["username"].as<String>();
    if (doc.containsKey("password")) tp_password = doc["password"].as<String>();
    file.close();
  }
}

void saveCredentials() {
  File file = SPIFFS.open("/creds.json", "w");
  if (file) {
    DynamicJsonDocument doc(512);
    doc["token"] = session_token;
    doc["time"] = millis();
    serializeJson(doc, file);
    file.close();
  }
}

void loadCredentials() {
  File file = SPIFFS.open("/creds.json", "r");
  if (file) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, file);
    if (doc.containsKey("token")) {
      session_token = doc["token"].as<String>();
      unsigned long saved_time = doc["time"];
      if ((millis() - saved_time) < (24 * 60 * 60 * 1000)) {
        authenticated = true;
      }
    }
    file.close();
  }
}

void saveWorkoutsToStorage() {
  File file = SPIFFS.open("/workouts.json", "w");
  if (file) {
    DynamicJsonDocument doc(8192);
    JsonArray workoutsArray = doc.createNestedArray("workouts");
    
    for (int i = 0; i < workout_count; i++) {
      JsonObject workoutObj = workoutsArray.createNestedObject();
      workoutObj["id"] = workouts[i].id;
      workoutObj["name"] = workouts[i].name;
      workoutObj["sport"] = workouts[i].sport;
      workoutObj["total_duration"] = workouts[i].total_duration;
      workoutObj["step_count"] = workouts[i].step_count;
      
      JsonArray stepsArray = workoutObj.createNestedArray("steps");
      for (int j = 0; j < workouts[i].step_count; j++) {
        JsonObject stepObj = stepsArray.createNestedObject();
        stepObj["name"] = workouts[i].steps[j].name;
        stepObj["duration"] = workouts[i].steps[j].duration;
        stepObj["intensity_type"] = workouts[i].steps[j].intensity_type;
        stepObj["target_min"] = workouts[i].steps[j].target_min;
        stepObj["target_max"] = workouts[i].steps[j].target_max;
        stepObj["zone"] = workouts[i].steps[j].zone;
        stepObj["step_type"] = workouts[i].steps[j].step_type;
      }
    }
    
    serializeJson(doc, file);
    file.close();
  }
}

void loadWorkoutsFromStorage() {
  File file = SPIFFS.open("/workouts.json", "r");
  if (file) {
    String content = file.readString();
    file.close();
    parseWorkouts(content);
  } else {
    // Create sample workout
    workout_count = 1;
    workouts[0].name = "Sample Easy Run";
    workouts[0].sport = "run";
    workouts[0].total_duration = 1800;
    workouts[0].step_count = 3;
    
    workouts[0].steps[0].name = "Warmup";
    workouts[0].steps[0].duration = 300;
    workouts[0].steps[0].intensity_type = "pace";
    workouts[0].steps[0].target_min = 6.0;
    workouts[0].steps[0].target_max = 6.5;
    
    workouts[0].steps[1].name = "Main";
    workouts[0].steps[1].duration = 1200;
    workouts[0].steps[1].intensity_type = "pace";
    workouts[0].steps[1].target_min = 5.0;
    workouts[0].steps[1].target_max = 5.3;
    
    workouts[0].steps[2].name = "Cooldown";
    workouts[0].steps[2].duration = 300;
    workouts[0].steps[2].intensity_type = "pace";
    workouts[0].steps[2].target_min = 6.0;
    workouts[0].steps[2].target_max = 7.0;
  }
}