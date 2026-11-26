#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* GPS module on UART1 (Serial2): TX=GP8, RX=GP9. */
/* See docs/gps-logger-design.md. NEO-series modules speak NMEA at */
/* 9600 bps by default; UBX configuration (higher rate, fewer */
/* sentences) comes later, once the module model is confirmed. */

GpsReport gpsReport;

static char lineBuf[128];
static size_t lineLen = 0;

bool initGPS() {
  gpsReport = {};
  lineLen = 0;
  Serial2.setTX(8);
  Serial2.setRX(9);
  Serial2.begin(9600);
  return true;
}

/* NMEA checksum: XOR of all characters between '$' and '*'. */
static bool checksumOk(const char *s) {
  const char *star = strchr(s, '*');
  if (!star) return false;
  unsigned char cs = 0;
  for (const char *p = s + 1; p < star; p++) cs ^= (unsigned char)*p;
  unsigned int expected = 0;
  if (sscanf(star + 1, "%2x", &expected) != 1) return false;
  return cs == (unsigned char)expected;
}

/* Copy field idx (0-based; field 0 is the "$GPRMC" header) into out. */
/* Returns false when the field is missing or empty. */
static bool getField(const char *s, int idx, char *out, size_t outSize) {
  const char *p = s;
  for (int i = 0; i < idx; i++) {
    p = strchr(p, ',');
    if (!p) return false;
    p++;
  }
  size_t n = 0;
  while (p[n] && p[n] != ',' && p[n] != '*' && n < outSize - 1) {
    out[n] = p[n];
    n++;
  }
  out[n] = '\0';
  return n > 0;
}

/* NMEA coordinate ddmm.mmmm (or dddmm.mmmm) -> decimal degrees. */
static double nmeaToDeg(double v) {
  int deg = (int)(v / 100.0);
  double minutes = v - deg * 100.0;
  return deg + minutes / 60.0;
}

static void printGpsLine() {
  /* One CSV line per RMC, alongside the Q lines from serial_output: */
  /* G,lat,lon,speed_kmh,course_deg,hh:mm:ss,fix,sats */
  char out[128];
  snprintf(out, sizeof(out), "G,%.7f,%.7f,%.2f,%.1f,%02u:%02u:%02u,%u,%u",
           gpsReport.lat_deg, gpsReport.lon_deg,
           (double)gpsReport.speed_kmh, (double)gpsReport.course_deg,
           gpsReport.hour, gpsReport.minute, gpsReport.second,
           gpsReport.fix ? 1u : 0u, (unsigned int)gpsReport.sats);
  Serial.println(out);
}

static void parseRMC(const char *s) {
  char buf[24];
  GpsReport &g = gpsReport;

  /* Field 1: UTC time hhmmss.ss */
  if (getField(s, 1, buf, sizeof(buf)) && strlen(buf) >= 6) {
    g.hour = (uint8_t)((buf[0] - '0') * 10 + (buf[1] - '0'));
    g.minute = (uint8_t)((buf[2] - '0') * 10 + (buf[3] - '0'));
    g.second = (uint8_t)((buf[4] - '0') * 10 + (buf[5] - '0'));
  }

  /* Field 2: status 'A' = valid fix, 'V' = void */
  char status = 0;
  if (getField(s, 2, buf, sizeof(buf))) status = buf[0];

  /* Fields 3-6: lat, N/S, lon, E/W (ddmm.mmmm / dddmm.mmmm) */
  char fLat[24], fNS[4], fLon[24], fEW[4];
  if (getField(s, 3, fLat, sizeof(fLat)) && getField(s, 4, fNS, sizeof(fNS)) &&
      getField(s, 5, fLon, sizeof(fLon)) && getField(s, 6, fEW, sizeof(fEW)) &&
      status == 'A') {
    double lat = nmeaToDeg(atof(fLat));
    if (fNS[0] == 'S') lat = -lat;
    double lon = nmeaToDeg(atof(fLon));
    if (fEW[0] == 'W') lon = -lon;
    g.lat_deg = lat;
    g.lon_deg = lon;
  }

  /* Field 7: speed over ground (knots) -> km/h */
  if (getField(s, 7, buf, sizeof(buf))) {
    g.speed_kmh = (float)(atof(buf) * 1.852);
  }
  /* Field 8: course over ground (degrees, true north) */
  if (getField(s, 8, buf, sizeof(buf))) {
    g.course_deg = (float)atof(buf);
  }

  /* Field 9: date ddmmyy */
  if (getField(s, 9, buf, sizeof(buf)) && strlen(buf) >= 6) {
    g.day = (uint8_t)((buf[0] - '0') * 10 + (buf[1] - '0'));
    g.month = (uint8_t)((buf[2] - '0') * 10 + (buf[3] - '0'));
    g.year = (uint16_t)(2000 + (buf[4] - '0') * 10 + (buf[5] - '0'));
  }

  g.fix = (status == 'A');
  if (g.fix) g.last_fix_ms = millis();
  g.updated = true;

  printGpsLine();
}

static void parseGGA(const char *s) {
  char buf[12];
  /* Field 7: satellites in use */
  if (getField(s, 7, buf, sizeof(buf))) {
    gpsReport.sats = (uint8_t)atoi(buf);
  }
}

static void handleSentence(const char *s) {
  if (strlen(s) < 6) return;
  if (!checksumOk(s)) return;
  /* Sentence type is at s[3..5] for any talker ($GP..., $GN..., ...). */
  if (strncmp(s + 3, "RMC", 3) == 0) {
    parseRMC(s);
  } else if (strncmp(s + 3, "GGA", 3) == 0) {
    parseGGA(s);
  }
}

void updateGPS() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (c == '$') {
      /* Start of a new sentence; resync even mid-line. */
      lineLen = 0;
      lineBuf[lineLen++] = c;
    } else if (c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleSentence(lineBuf);
        lineLen = 0;
      }
    } else if (c == '\r') {
      /* Ignore CR; sentences end with CRLF. */
    } else if (lineLen > 0) {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0; /* Overlong line: drop the sentence. */
      }
    }
  }
}
