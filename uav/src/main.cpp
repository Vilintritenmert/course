#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
#include <cstring>
#include <cfloat>
#include <string>

using json = nlohmann::json;
using namespace std;

// ── #define macros ────────────────────────────────────────────────────────

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

// ── constants & enum ──────────────────────────────────────────────────────

const float G = 9.81f;
const int MAX_STEPS = 10000;

enum DroneState { STOPPED = 0, ACCELERATING = 1, DECELERATING = 2, TURNING = 3, MOVING = 4 };

// ── structs ───────────────────────────────────────────────────────────────

struct Coord {
  float x, y;
  Coord operator+(const Coord& o) const { return {x + o.x, y + o.y}; }
  Coord operator-(const Coord& o) const { return {x - o.x, y - o.y}; }
  Coord operator*(float s) const { return {x * s, y * s}; }
  Coord operator/(float s) const { return {x / s, y / s}; }
  bool operator==(const Coord& o) const { return x == o.x && y == o.y; }
};

float length(Coord c)
{
  return sqrtf(c.x * c.x + c.y * c.y);
}
Coord normalize(Coord c)
{
  float l = length(c);
  return l > 1e-9f ? c / l : Coord{0.0f, 0.0f};
}

struct AmmoParams {
  char name[32];
  float mass, drag, lift;

  AmmoParams operator=(const json& jsonData)
  {
    strncpy(name, jsonData["name"].get<std::string>().c_str(), 31);
    name[31] = '\0';
    mass = jsonData["mass"];
    drag = jsonData["drag"];
    lift = jsonData["lift"];

    return *this;
  }
};

struct DroneConfig {
  Coord startPos;
  float altitude, initialDir, attackSpeed, accelPath;
  char ammoName[32];
  float arrayTimeStep, simTimeStep, hitRadius, angularSpeed, turnThreshold;

  DroneConfig operator=(const json& jsonData)
  {
    try {
      startPos.x = jsonData["drone"]["position"]["x"];
      startPos.y = jsonData["drone"]["position"]["y"];
      altitude = jsonData["drone"]["altitude"];
      initialDir = jsonData["drone"]["initialDirection"];
      attackSpeed = jsonData["drone"]["attackSpeed"];
      accelPath = jsonData["drone"]["accelerationPath"];
      angularSpeed = jsonData["drone"]["angularSpeed"];
      turnThreshold = jsonData["drone"]["turnThreshold"];
      arrayTimeStep = jsonData["targetArrayTimeStep"];
      simTimeStep = jsonData["simulation"]["timeStep"];
      hitRadius = jsonData["simulation"]["hitRadius"];
      strncpy(ammoName, jsonData["ammo"].get<std::string>().c_str(), 31);
      ammoName[31] = '\0';

      return *this;
    }
    catch (const exception& e) {
      throw runtime_error("Invalid config.json: " + string(e.what()));
    }
  }
};

struct SimStep {
  Coord pos;
  float direction;
  int state, targetIdx;
  Coord dropPoint, aimPoint, predictedTarget;
};

// ── helpers ───────────────────────────────────────────────────────────────

const AmmoParams* findAmmo(const vector<AmmoParams>& ammoConfig, const char* name)
{
  for (const AmmoParams& param : ammoConfig)
    if (strcmp(name, param.name) == 0)
      return &param;
  return nullptr;
}

float normalizeAngle(float a)
{
  while (a > (float)M_PI)
    a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI)
    a += 2.0f * (float)M_PI;
  return a;
}

// ── target interpolation ──────────────────────────────────────────────────

Coord interpolateTarget(const Coord* path, int timeSteps, float t, float arrayTimeStep)
{
  int rawIdx = (int)floorf(t / arrayTimeStep);
  int idx = rawIdx % timeSteps;
  int next = (idx + 1) % timeSteps;
  float frac = (t - rawIdx * arrayTimeStep) / arrayTimeStep;

  return path[idx] + (path[next] - path[idx]) * frac;
}

// ── ballistics (Cardano + power series from HW1) ──────────────────────────

float computeFlightTime(float zd, float m, float d, float l, float v)
{
  float a = d * G * m - 2.0f * d * d * l * v;
  float b = -3.0f * G * m * m + 3.0f * d * l * m * v;
  float c = 6.0f * m * m * zd;

  if (fabsf(a) < 1e-9f)
    return sqrtf(2.0f * zd / G);

  float p = -b * b / (3.0f * a * a);
  float q = 2.0f * b * b * b / (27.0f * a * a * a) + c / a;

  if (p >= 0.0f)
    return sqrtf(2.0f * zd / G);

  float inner = 3.0f * q / (2.0f * p) * sqrtf(-3.0f / p);
  inner = fmaxf(-1.0f, fminf(1.0f, inner));
  float phi = acosf(inner);
  float u = 2.0f * sqrtf(-p / 3.0f) * cosf((phi + 4.0f * (float)M_PI) / 3.0f);
  float t = u - b / (3.0f * a);
  return (t > 0.0f) ? t : sqrtf(2.0f * zd / G);
}

float computeHorizDist(float t, float v, float m, float d, float l)
{
  float l2 = l * l, l4 = l2 * l2;
  float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

  float r1 = v * t - t2 * d * v / (2.0f * m);
  float r2 = t3 * (6.0f * d * G * l * m - 6.0f * d * d * (l2 - 1.0f) * v) / (36.0f * m * m);
  float r3 = 0.0f, r4 = 0.0f;

  if (l > 1e-6f) {
    float dn3 = 36.0f * (1.0f + l2) * (1.0f + l2) * m * m * m;
    r3 =
      t4 *
      (-6.0f * d * d * G * l * (1.0f + l2 + l4) * m + 3.0f * d * d * d * l2 * (1.0f + l2) * v + 6.0f * d * d * d * l4 * (1.0f + l2) * v) /
      dn3;
    float dn4 = 36.0f * (1.0f + l2) * m * m * m * m;
    r4 = t5 * (3.0f * d * d * d * G * l * l2 * m - 3.0f * d * d * d * d * l2 * (1.0f + l2) * v) / dn4;
  }
  return r1 + r2 + r3 + r4;
}

// ── drop point: h metres before target along drone→target line ────────────

bool computeDropPoint(Coord tgt, Coord drone, float h, Coord& drop)
{
  Coord d = tgt - drone;
  float dist = length(d);
  if (dist < 1e-3f)
    return false;
  drop = drone + d * ((dist - h) / dist);
  return true;
}

// ── lead targeting (one-pass) ─────────────────────────────────────────────

bool computeLeadDrop(const Coord* path,
                     int timeSteps,
                     float currentTime,
                     Coord dronePos,
                     float zd,
                     float bm,
                     float bd,
                     float bl,
                     float attackSpeed,
                     float arrayTimeStep,
                     Coord& dropPos,
                     Coord& predPos,
                     float& totalTime)
{
  float tFlight = computeFlightTime(zd, bm, bd, bl, attackSpeed);
  float h = computeHorizDist(tFlight, attackSpeed, bm, bd, bl);

  // Predict target at landing time (droneTime=0 avoids overshoot when target approaches)
  predPos = interpolateTarget(path, timeSteps, currentTime + tFlight, arrayTimeStep);

  if (!computeDropPoint(predPos, dronePos, h, dropPos))
    dropPos = dronePos;

  float distToPred = length(predPos - dronePos);
  totalTime = fmaxf(0.0f, distToPred - h) / attackSpeed;
  return true;
}

// ── target selection ──────────────────────────────────────────────────────

int selectTarget(Coord dronePos,
                 float droneSpeed,
                 DroneState state,
                 float accel,
                 float attackSpeed,
                 float currentTime,
                 float zd,
                 float bm,
                 float bd,
                 float bl,
                 float arrayTimeStep,
                 Coord** targets,
                 int timeSteps,
                 int targetCount,
                 int currentTargetIdx,
                 float turnTimeLeft,
                 Coord outDrop[],
                 Coord outPred[])
{
  float timeToStop = 0.0f;
  switch (state) {
    case ACCELERATING:
      timeToStop = droneSpeed / accel;
      break;
    case MOVING:
      timeToStop = attackSpeed / accel;
      break;
    case DECELERATING:
      timeToStop = droneSpeed / accel;
      break;
    case TURNING:
      timeToStop = turnTimeLeft;
      break;
    default:
      timeToStop = 0.0f;
  }

  int best = -1;
  float bestTime = FLT_MAX;

  for (int i = 0; i < targetCount; i++) {
    Coord dPos, pPos;
    float tTime;
    if (!computeLeadDrop(targets[i], timeSteps, currentTime, dronePos, zd, bm, bd, bl, attackSpeed, arrayTimeStep, dPos, pPos, tTime))
      continue;
    outDrop[i] = dPos;
    outPred[i] = pPos;
    float effective = tTime + (i != currentTargetIdx ? timeToStop : 0.0f);
    if (effective < bestTime) {
      bestTime = effective;
      best = i;
    }
  }
  return best;
}

// ── drone state machine (one time step) ──────────────────────────────────

void updateDrone(Coord& pos,
                 float& dir,
                 float& speed,
                 DroneState& state,
                 float desiredDir,
                 float dt,
                 float attackSpeed,
                 float accel,
                 float angularSpeed,
                 float turnThreshold,
                 float& turnAngleLeft)
{
  float angleDiff = normalizeAngle(desiredDir - dir);
  float angStep = angularSpeed * dt;

  switch (state) {
    case STOPPED:
      speed = 0.0f;
      if (fabsf(angleDiff) > turnThreshold) {
        state = TURNING;
        turnAngleLeft = fabsf(angleDiff);
      }
      else {
        state = ACCELERATING;
      }
      break;

    case ACCELERATING:
      if (fabsf(angleDiff) > turnThreshold) {
        state = DECELERATING;
      }
      else {
        dir += (fabsf(angleDiff) <= angStep) ? angleDiff : (angleDiff > 0 ? angStep : -angStep);
        dir = normalizeAngle(dir);
        speed += accel * dt;
        if (speed >= attackSpeed) {
          speed = attackSpeed;
          state = MOVING;
        }
      }
      pos = pos + Coord{cosf(dir), sinf(dir)} * (speed * dt);
      break;

    case MOVING:
      if (fabsf(angleDiff) > turnThreshold) {
        state = DECELERATING;
      }
      else {
        dir += (fabsf(angleDiff) <= angStep) ? angleDiff : (angleDiff > 0 ? angStep : -angStep);
        dir = normalizeAngle(dir);
      }
      pos = pos + Coord{cosf(dir), sinf(dir)} * (speed * dt);
      break;

    case DECELERATING:
      speed -= accel * dt;
      if (speed <= 0.0f) {
        speed = 0.0f;
        state = TURNING;
        turnAngleLeft = fabsf(angleDiff);
      }
      else {
        pos = pos + Coord{cosf(dir), sinf(dir)} * (speed * dt);
      }
      break;

    case TURNING:
      if (fabsf(angleDiff) <= angStep) {
        dir = desiredDir;
        state = ACCELERATING;
        turnAngleLeft = 0.0f;
      }
      else {
        dir += (angleDiff > 0 ? angStep : -angStep);
        dir = normalizeAngle(dir);
        turnAngleLeft = fabsf(angleDiff) - angStep;
        if (turnAngleLeft < 0.0f)
          turnAngleLeft = 0.0f;
      }
      break;
  }
}

json loadJsonFile(const string& path)
{
  ifstream fc(path);
  if (!fc.is_open()) {
    throw runtime_error("Cannot open " + path);
  }
  json jc;
  fc >> jc;
  fc.close();

  return jc;
}

// ── main ─────────────────────────────────────────────────────────────────

void storeJson(const string& fileName, const json& output)
{
  ofstream fout(fileName);
  if (!fout.is_open()) {
    throw runtime_error("Cannot write to " + fileName);
  }
  else {
    fout << output.dump(2);
    fout.close();
  }
}

int main(int argc, char** argv)
{
  try {
    if (argc != 2) {
      throw invalid_argument("usage: uav_calculator <data_folder_path>\n");
    }

    const string dataFolderName = argv[1];

    DroneConfig droneCfg;
    droneCfg = loadJsonFile(dataFolderName + string("/config.json"));

    // Ammo Config Loading
    json jsonAmmoConfig = loadJsonFile(dataFolderName + string("/ammo.json"));
    vector<AmmoParams> ammoConfig(jsonAmmoConfig.size());
    for (const json& ammoRawData : jsonAmmoConfig) {
      AmmoParams ammoItem;
      ammoItem = ammoRawData;
      ammoConfig.push_back(ammoItem);
    }

    // Targets Loading
    json jsonTargets = loadJsonFile(dataFolderName + string("/targets.json"));
    int targetCount = jsonTargets["targetCount"];
    int timeSteps = jsonTargets["timeSteps"];
    Coord** targets = new Coord*[targetCount];
    for (int i = 0; i < targetCount; i++) {
      targets[i] = new Coord[timeSteps];
      for (int t = 0; t < timeSteps; t++) {
        targets[i][t].x = jsonTargets["targets"][i]["positions"][t]["x"];
        targets[i][t].y = jsonTargets["targets"][i]["positions"][t]["y"];
      }
    }
    
    LOG("Target loaded: " << targetCount << endl );

    const AmmoParams* ammoParams = findAmmo(ammoConfig, droneCfg.ammoName);
    if (ammoParams == nullptr) {
      throw runtime_error("Ammo not found: " + string(droneCfg.ammoName));
    }
    float bm = ammoParams->mass;
    float bd = ammoParams->drag;
    float bl = ammoParams->lift;
    LOG("Ammo found: " << ammoParams->name << " mass=" << bm);

    // ── Validate parameters ───────────────────────────────────────────────
    if (droneCfg.attackSpeed <= 0.0f || droneCfg.accelPath <= 0.0f || droneCfg.altitude <= 0.0f) {
      throw runtime_error("ERROR: Invalid parameters\n");
    }

    float accel = droneCfg.attackSpeed * droneCfg.attackSpeed / (2.0f * droneCfg.accelPath);
    float h = computeHorizDist(computeFlightTime(droneCfg.altitude, bm, bd, bl, droneCfg.attackSpeed), droneCfg.attackSpeed, bm, bd, bl);

    // ── Allocate simulation arrays ────────────────────────────────────────
    SimStep* steps = new SimStep[MAX_STEPS + 1];
    Coord* allDrop = new Coord[targetCount];
    Coord* allPred = new Coord[targetCount];

    // ── Simulation state ──────────────────────────────────────────────────
    Coord dronePos = droneCfg.startPos;
    float droneDir = droneCfg.initialDir;
    float droneSpeed = 0.0f;
    DroneState droneState = STOPPED;
    int currentTarget = -1;
    float turnAngleLeft = 0.0f;
    float currentTime = 0.0f;
    int step = 0;
    bool stagingMode = false;

    // ── Main loop ─────────────────────────────────────────────────────────
    while (step < MAX_STEPS) {
      float turnTimeLeft = (droneState == TURNING) ? (turnAngleLeft / droneCfg.angularSpeed) : 0.0f;

      int best = selectTarget(dronePos,
                              droneSpeed,
                              droneState,
                              accel,
                              droneCfg.attackSpeed,
                              currentTime,
                              droneCfg.altitude,
                              bm,
                              bd,
                              bl,
                              droneCfg.arrayTimeStep,
                              targets,
                              timeSteps,
                              targetCount,
                              currentTarget,
                              turnTimeLeft,
                              allDrop,
                              allPred);
      if (best == -1)
        break;

      currentTarget = best;
      Coord predPos = allPred[currentTarget];

      // Record this step
      steps[step].pos = dronePos;
      steps[step].direction = droneDir;
      steps[step].state = (int)droneState;
      steps[step].targetIdx = currentTarget;
      steps[step].dropPoint = allDrop[currentTarget];
      steps[step].predictedTarget = predPos;
      steps[step].aimPoint = dronePos + Coord{cosf(droneDir), sinf(droneDir)} * h;

      DEBUG("Step " << step << " pos=(" << dronePos.x << "," << dronePos.y << ") target=" << currentTarget << " state=" << (int)droneState);

      float distToPred = length(predPos - dronePos);

      // Staging update must run before drop check
      if (distToPred < h)
        stagingMode = true;
      else if (distToPred >= h + droneCfg.accelPath)
        stagingMode = false;

      // Drop check: only at cruise speed, in approach mode, close enough
      if (!stagingMode && droneState == MOVING && distToPred <= h + droneCfg.hitRadius) {
        step++;
        break;
      }

      // Navigation target: stage point when too close, else direct approach
      Coord navPos;
      if (stagingMode && distToPred > 1e-3f)
        navPos = predPos + normalize(dronePos - predPos) * (h + droneCfg.accelPath);
      else
        navPos = predPos;

      float desiredDir = atan2f(navPos.y - dronePos.y, navPos.x - dronePos.x);
      updateDrone(dronePos,
                  droneDir,
                  droneSpeed,
                  droneState,
                  desiredDir,
                  droneCfg.simTimeStep,
                  droneCfg.attackSpeed,
                  accel,
                  droneCfg.angularSpeed,
                  droneCfg.turnThreshold,
                  turnAngleLeft);

      currentTime += droneCfg.simTimeStep;
      step++;
    }

    int totalSteps = step;
    LOG("Simulation complete. Steps: " << totalSteps << " Target: " << currentTarget);

    // ── Write simulation.json ─────────────────────────────────────────────
    json output;
    output["totalSteps"] = totalSteps;
    output["steps"] = json::array();
    for (int i = 0; i < totalSteps; i++) {
      json s;
      s["position"] = {{"x", steps[i].pos.x}, {"y", steps[i].pos.y}};
      s["direction"] = steps[i].direction;
      s["state"] = steps[i].state;
      s["targetIndex"] = steps[i].targetIdx;
      s["dropPoint"] = {{"x", steps[i].dropPoint.x}, {"y", steps[i].dropPoint.y}};
      s["aimPoint"] = {{"x", steps[i].aimPoint.x}, {"y", steps[i].aimPoint.y}};
      s["predictedTarget"] = {{"x", steps[i].predictedTarget.x}, {"y", steps[i].predictedTarget.y}};
      output["steps"].push_back(s);
    }
    storeJson(argv[1] + string("/simulation.json"), output);

    // ── Free dynamic memory ───────────────────────────────────────────────
    delete[] steps;
    steps = nullptr;
    delete[] allDrop;
    allDrop = nullptr;
    delete[] allPred;
    allPred = nullptr;
    for (int i = 0; i < targetCount; i++) {
      delete[] targets[i];
      targets[i] = nullptr;
    }
    delete[] targets;
    targets = nullptr;

    return 0;
  }
  catch (const exception& e) {
    cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}