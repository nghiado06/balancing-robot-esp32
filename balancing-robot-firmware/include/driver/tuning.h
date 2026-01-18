#ifndef RETO_UI_H
#define RETO_UI_H

#include <Arduino.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFi.h>

static const byte DNS_PORT = 53;

/*********************************************************************************************************
 ========================================= [REMOTE CONTROL UI] ===========================================
 ********************************************************************************************************/

class ControlUI
{
public:
    // Constructor
    ControlUI();
    // Initialize remote control (AP + WebServer)
    void begin();

    // Process signals in loop()
    void remoteProcess();

    // Set names for 8 function buttons
    void setName(int functionIndex, const String &name);

    // Direction - event 1 time
    bool isUpPressed();
    bool isDownPressed();
    bool isLeftPressed();
    bool isRightPressed();

    // Direction - held
    bool isUpHeld();
    bool isDownHeld();
    bool isLeftHeld();
    bool isRightHeld();

    // Function - event 1 time
    bool isFunction1Pressed();
    bool isFunction2Pressed();
    bool isFunction3Pressed();
    bool isFunction4Pressed();
    bool isFunction5Pressed();
    bool isFunction6Pressed();
    bool isFunction7Pressed();
    bool isFunction8Pressed();

    // Function - held
    bool isFunction1Held();
    bool isFunction2Held();
    bool isFunction3Held();
    bool isFunction4Held();
    bool isFunction5Held();
    bool isFunction6Held();
    bool isFunction7Held();
    bool isFunction8Held();

    void logAppend(const String &text); // History (append time-stamped line)
    void clearHistory();                // Clear history

    void addLog(const String &text, uint8_t position);

private:
    WebServer *server = nullptr;
    DNSServer *dnsServer = nullptr;

    // State variables
    volatile bool upPressed, downPressed, leftPressed, rightPressed;
    volatile bool upHeld, downHeld, leftHeld, rightHeld;

    volatile bool funcPressed[8];
    volatile bool funcHeld[8];

    String funcName[8];

    // Internal
    String buildPage();
    void handleRoot();
    void handleBtn();
    void handleNotFound();

    // History Logging Helpers
    static const uint16_t HIST_CAP = 200; // Max 200 lines of history
    String histBuf[HIST_CAP];
    uint16_t histStart = 0, histSize = 0;
    void pushHist_(const String &line);
    String buildHistory_();    // Return HISTORY
    void handleHistory();      // GET /history
    void handleClearHistory(); // GET /clearHistory

    // === Panel status (4 group: F1-2, F3-4, F5-6, F7-8) ===
    static const uint8_t PANEL_COUNT = 4;
    String panelText[PANEL_COUNT]; // each panel text

    String buildPanels_(); // Return all panels (one line each)
    void handlePanels();   // GET /panels

    String formatMillis_(uint32_t ms);
};

#endif // RETO_UI_H