#include <iostream>
#include <fstream>
#include <cstring>
#include <stdexcept>
#include <cmath>
#include <ostream>

using namespace std;

int main()
{
  try {
    cout << "Reading input data " << endl;

    float xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath;
    char ammoName[15] = "";
    float m, d, l = 0;

    fstream inputFile("./input.txt");
    if (!inputFile.is_open()) {
      throw invalid_argument("`input.txt` file not found");
    }

    inputFile >> xd >> yd >> zd >> targetX >> targetY >> attackSpeed >> accelerationPath;
    inputFile >> ammoName;

    if (strcmp(ammoName, "VOG-17") == 0) {
      m = 0.35;
      d = 0.07;
      l = 0.;
    }
    else if (strcmp(ammoName, "M67") == 0) {
      m = 0.6;
      d = 0.1;
      l = 0.;
    }
    else if (strcmp(ammoName, "RKG-3") == 0) {
      m = 1.2;
      d = 0.1;
      l = 0.;
    }
    else if (strcmp(ammoName, "GLIDING-VOG") == 0) {
      m = 0.45;
      d = 0.1;
      l = 1.;
    }
    else if (strcmp(ammoName, "GLIDING-RKG") == 0) {
      m = 1.4;
      d = 0.1;
      l = 1.;
    }
    else {
      throw invalid_argument("ammo_name is not acceptable");
    }

    float g = 9.81;

    float v0 = attackSpeed;
    float z0 = zd;

    float a = d * g * m - 2 * d * d * l * v0;
    float b = -3 * g * m * m + 3 * d * l * m * v0;
    float c = 6 * m * m * z0;

    float p = -b * b / (3 * a * a);
    float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

    printf("a = %f b = %f c = %f \n", a, b, c);
    printf("p = %f q = %f \n", p, q);

    if ((p >= 0) || (fabs(3 * q / (2 * p) * sqrt(-3 / p)) > 1)) {
      throw invalid_argument("Has no solution");
    }

    float phi = acos(3 * q / (2 * p) * sqrt(-3 / p));

    float t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

    printf("phi = %f t = %f \n", phi, t);

    double hightDistance =
      (attackSpeed * t) - (pow(t, 2) * d * attackSpeed) / (2.0 * m) +
      (pow(t, 3) * (6.0 * d * g * l * m - 6.0 * pow(d, 2) * (pow(l, 2) - 1.0) * attackSpeed)) / (36.0 * pow(m, 2)) +
      (pow(t, 4) *
       (-6.0 * pow(d, 2) * g * l * (1.0 + pow(l, 2) + pow(l, 4)) * m + 3.0 * pow(d, 3) * pow(l, 2) * (1.0 + pow(l, 2)) * attackSpeed +
        6.0 * pow(d, 3) * pow(l, 4) * (1.0 + pow(l, 2)) * attackSpeed)) /
        (36.0 * pow(1.0 + pow(l, 2), 2) * pow(m, 3)) +
      (pow(t, 5) * (3.0 * pow(d, 3) * g * pow(l, 3) * m - 3.0 * pow(d, 4) * pow(l, 2) * (1.0 + pow(l, 2)) * attackSpeed)) /
        (36.0 * (1.0 + pow(l, 2)) * pow(m, 4));

    cout << "HightDistance :" << hightDistance << endl;

    float distanceToTarget = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));

    bool isCloseDistance = hightDistance + accelerationPath > distanceToTarget;

    ofstream outputFile("./output.txt");
    if (!outputFile.is_open()) {
      throw invalid_argument("`output.txt` file is not available");
    }

    if (isCloseDistance) {
      outputFile << xd << " " << yd << endl;
      if (fabs(distanceToTarget) < 1e-6) {
        xd = targetX - (hightDistance + accelerationPath);
        yd = targetY;
        distanceToTarget = hightDistance + accelerationPath;
      }
      else {
        xd = targetX - (targetX - xd) * (hightDistance + accelerationPath) / distanceToTarget;
        yd = targetY - (targetY - yd) * (hightDistance + accelerationPath) / distanceToTarget;
        distanceToTarget = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));
      }
    }

    float ration = (distanceToTarget - hightDistance) / distanceToTarget;
    float fireX = xd + (targetX - xd) * ration;
    float fireY = yd + (targetY - yd) * ration;

    outputFile << fireX << " " << fireY << endl;

    cout << "Data calculated succesfully" << endl;

    return 0;
  }
  catch (const invalid_argument &e) {
    cerr << "Calculation failed with the reason: " << e.what() << endl;

    return 1;
  }
}