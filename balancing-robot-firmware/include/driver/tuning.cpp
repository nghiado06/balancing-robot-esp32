#include "tuning.h"

/*********************************************************************************************************
 ============================================ [ REMOTE CONTROL ] =========================================
 ********************************************************************************************************/
/*********************************************************************************************************
 * @brief   Constructor
 ********************************************************************************************************/
ControlUI::ControlUI()
{
    server = new WebServer(80);
    dnsServer = new DNSServer();

    upPressed = downPressed = leftPressed = rightPressed = false;
    upHeld = downHeld = leftHeld = rightHeld = false;

    for (int i = 0; i < 8; i++)
    {
        funcPressed[i] = false;
        funcHeld[i] = false;
        funcName[i] = "F" + String(i + 1);
    }
}

/*********************************************************************************************************
 * @brief   Event Pressed
 * @details These functions check if a specific button was pressed since the last check. If it was pressed,
 *          the function returns true and resets the pressed state to false.
 *********************************************************************************************************/
bool ControlUI::isUpPressed()
{
    bool t = upPressed;
    upPressed = false;
    return t;
}
bool ControlUI::isDownPressed()
{
    bool t = downPressed;
    downPressed = false;
    return t;
}
bool ControlUI::isLeftPressed()
{
    bool t = leftPressed;
    leftPressed = false;
    return t;
}
bool ControlUI::isRightPressed()
{
    bool t = rightPressed;
    rightPressed = false;
    return t;
}

bool ControlUI::isFunction1Pressed()
{
    bool t = funcPressed[0];
    funcPressed[0] = false;
    return t;
}
bool ControlUI::isFunction2Pressed()
{
    bool t = funcPressed[1];
    funcPressed[1] = false;
    return t;
}
bool ControlUI::isFunction3Pressed()
{
    bool t = funcPressed[2];
    funcPressed[2] = false;
    return t;
}
bool ControlUI::isFunction4Pressed()
{
    bool t = funcPressed[3];
    funcPressed[3] = false;
    return t;
}
bool ControlUI::isFunction5Pressed()
{
    bool t = funcPressed[4];
    funcPressed[4] = false;
    return t;
}
bool ControlUI::isFunction6Pressed()
{
    bool t = funcPressed[5];
    funcPressed[5] = false;
    return t;
}
bool ControlUI::isFunction7Pressed()
{
    bool t = funcPressed[6];
    funcPressed[6] = false;
    return t;
}
bool ControlUI::isFunction8Pressed()
{
    bool t = funcPressed[7];
    funcPressed[7] = false;
    return t;
}

/*********************************************************************************************************
 * @brief   Event Pressed
 * @details These functions check if a specific button was pressed since the last check. If it was pressed,
 *          the function returns true and resets the pressed state to false.
 ********************************************************************************************************/
bool ControlUI::isUpHeld() { return upHeld; }
bool ControlUI::isDownHeld() { return downHeld; }
bool ControlUI::isLeftHeld() { return leftHeld; }
bool ControlUI::isRightHeld() { return rightHeld; }

bool ControlUI::isFunction1Held() { return funcHeld[0]; }
bool ControlUI::isFunction2Held() { return funcHeld[1]; }
bool ControlUI::isFunction3Held() { return funcHeld[2]; }
bool ControlUI::isFunction4Held() { return funcHeld[3]; }
bool ControlUI::isFunction5Held() { return funcHeld[4]; }
bool ControlUI::isFunction6Held() { return funcHeld[5]; }
bool ControlUI::isFunction7Held() { return funcHeld[6]; }
bool ControlUI::isFunction8Held() { return funcHeld[7]; }

/*********************************************************************************************************
 * @brief   Set Function Name
 * @details This function sets the name of a function button for display in the HTML UI.
 *
 * @param[in] functionIndex Index of the function button (1-8)
 * @param[in] name Name to set for the function button
 ********************************************************************************************************/
void ControlUI::setName(int functionIndex, const String &name)
{
    if (functionIndex < 1 || functionIndex > 8)
        return;
    funcName[functionIndex - 1] = name;
}

/*********************************************************************************************************
 * @brief   Build Page
 * @details This function builds the HTML page for the remote control interface, including styles and
 *          button elements.
 *
 * @return  HTML page as a String
 ********************************************************************************************************/
String ControlUI::buildPage()
{
    String html =
        "<!DOCTYPE html><html lang='en'><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'>"
        "<title>RETOCAR</title>"
        "<style>"
        /* --- Logging (history only) --- */
        ".logwrap{width:min(820px,94vw);margin:8px auto 12px;display:flex;flex-direction:column;gap:6px;}"
        ".pane{display:flex;flex-direction:column;gap:6px;}"
        ".logheader{font-weight:700;font-size:14px;}"
        ".log-gray{color:#111827;}" /* LOGGING HISTORY title */
        ".logbox{height:160px;border:2px solid #e5e7eb;border-radius:10px;padding:10px;margin:0;"
        " background:#0b1020;color:#e5e7eb;overflow:auto;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;"
        " font-size:12px;line-height:1.4;}"
        ".logbar{display:flex;justify-content:flex-end;gap:8px;}"
        ".logbtn{border:2px solid #2563eb;background:transparent;color:#2563eb;border-radius:10px;"
        " padding:6px 12px;font-weight:600;cursor:pointer;}"
        ".logbtn:active{background:#eff6ff;}"

        /* --- Page & pads --- */
        "body{margin:0;font-family:system-ui,-apple-system,BlinkMacSystemFont,sans-serif;"
        "background:#ffffff;color:#111827;display:flex;flex-direction:column;align-items:center;min-height:100vh;}"
        ".title{margin:10px 0 4px;font-size:32px;font-weight:800;letter-spacing:3px;}"
        ".title span:nth-child(1){color:#fbbf24;}"
        ".title span:nth-child(2){color:#3b82f6;}"
        ".hint{font-size:10px;color:#9ca3af;margin-bottom:4px;}"
        ".wrapper{display:flex;gap:32px;align-items:center;justify-content:center;flex-wrap:wrap;padding:4px 8px;}"
        ".pad-left{width:200px;height:200px;position:relative;}"
        ".d-btn{width:64px;height:64px;border-radius:18px;border:4px solid #2563eb;"
        "background:transparent;cursor:pointer;transition:0.06s;box-sizing:border-box;"
        "font-size:26px;font-weight:700;color:#2563eb;"
        "display:flex;align-items:center;justify-content:center;"
        "touch-action:none;-webkit-user-select:none;user-select:none;}"
        ".d-btn.active{background:#eff6ff;}"
        ".d-up{position:absolute;top:4px;left:68px;}"
        ".d-left{position:absolute;top:68px;left:4px;}"
        ".d-right{position:absolute;top:68px;right:4px;}"
        ".d-down{position:absolute;bottom:4px;left:68px;}"

        /* --- Right pad: groups with 2 buttons + panel --- */
        ".pad-right{display:flex;flex-direction:column;gap:10px;"
        " width:260px;max-width:94vw;}"

        ".func-group{border:2px solid #e5e7eb;border-radius:16px;padding:8px;"
        " display:flex;flex-direction:column;gap:6px;box-sizing:border-box;}"

        ".func-row{display:flex;gap:10px;}"

        ".func-panel{height:32px;border-radius:12px;border:2px solid #e5e7eb;"
        " font-size:12px;color:#111827;background:#f9fafb;"
        " display:flex;align-items:center;justify-content:center;text-align:center;"
        " padding:0 6px;box-sizing:border-box;}"

        /* --- Function buttons --- */
        ".f-btn{border-radius:16px;border:4px solid #2563eb;background:transparent;"
        "font-size:14px;font-weight:600;color:#2563eb;cursor:pointer;"
        "transition:0.06s;box-sizing:border-box;"
        "display:flex;align-items:center;justify-content:center;"
        "flex:1 1 0;min-width:0;"
        "touch-action:none;-webkit-user-select:none;user-select:none;}"
        ".f-btn.active{background:#eff6ff;}"
        ".f-btn.yellow{border-color:#fbbf24;color:#fbbf24;}"
        ".f-btn.yellow.active{background:#fffbeb;}"

        ".status{margin-top:4px;font-size:10px;color:#6b7280;min-height:12px;}"

        "@media (orientation:landscape){ .wrapper{flex-direction:row;gap:40px;} }"
        "</style>"
        "</head><body>"
        "<div class='title'><span>RETO</span><span>CAR</span></div>"
        "<div class='hint'>Rotate your phone sideways for gamepad mode</div>"
        "<div class='wrapper'>"
        "  <div class='pad-left'>"
        "    <button class='d-btn d-up'    id='btn-up'    >&#9650;</button>"
        "    <button class='d-btn d-left'  id='btn-left'  >&#9664;</button>"
        "    <button class='d-btn d-right' id='btn-right' >&#9654;</button>"
        "    <button class='d-btn d-down'  id='btn-down'  >&#9660;</button>"
        "  </div>"
        "  <div class='pad-right'>";

    // Group 1: F1, F2 + panel-0
    html +=
        "    <div class='func-group'>"
        "      <div class='func-row'>"
        "        <button class='f-btn' id='btn-f1'>";
    html += funcName[0];
    html += "</button>"
            "        <button class='f-btn' id='btn-f2'>";
    html += funcName[1];
    html += "</button>"
            "      </div>"
            "      <div class='func-panel' id='panel-0'>&nbsp;</div>"
            "    </div>";

    // Group 2: F3, F4 + panel-1 (yellow)
    html +=
        "    <div class='func-group'>"
        "      <div class='func-row'>"
        "        <button class='f-btn yellow' id='btn-f3'>";
    html += funcName[2];
    html += "</button>"
            "        <button class='f-btn yellow' id='btn-f4'>";
    html += funcName[3];
    html += "</button>"
            "      </div>"
            "      <div class='func-panel' id='panel-1'>&nbsp;</div>"
            "    </div>";

    // Group 3: F5, F6 + panel-2
    html +=
        "    <div class='func-group'>"
        "      <div class='func-row'>"
        "        <button class='f-btn' id='btn-f5'>";
    html += funcName[4];
    html += "</button>"
            "        <button class='f-btn' id='btn-f6'>";
    html += funcName[5];
    html += "</button>"
            "      </div>"
            "      <div class='func-panel' id='panel-2'>&nbsp;</div>"
            "    </div>";

    // Group 4: F7, F8 + panel-3 (yellow)
    html +=
        "    <div class='func-group'>"
        "      <div class='func-row'>"
        "        <button class='f-btn yellow' id='btn-f7'>";
    html += funcName[6];
    html += "</button>"
            "        <button class='f-btn yellow' id='btn-f8'>";
    html += funcName[7];
    html += "</button>"
            "      </div>"
            "      <div class='func-panel' id='panel-3'>&nbsp;</div>"
            "    </div>";

    html +=
        "  </div>" // pad-right
        "</div>"   // wrapper
        "<div class='status' id='statusText'></div>"
        "<div class='logwrap'>"
        "  <div class='pane'>"
        "    <div class='logheader log-gray'>LOGGING HISTORY</div>"
        "    <pre id='logbox-hist' class='logbox' spellcheck='false'></pre>"
        "    <div class='logbar'>"
        "      <button class='logbtn' id='clear-hist'>Clear</button>"
        "    </div>"
        "  </div>"
        "</div>"
        "<script>"
        "function sendCmd(c){"
        " fetch('/btn?b='+c)"
        "  .then(r=>r.text())"
        "  .then(t=>{document.getElementById('statusText').textContent=t;})"
        "  .catch(e=>{document.getElementById('statusText').textContent='Conn err';});"
        "}"
        "function bindHold(id,key){"
        " const el=document.getElementById(id);"
        " const press=(ev)=>{ev.preventDefault();el.classList.add('active');sendCmd(key+'_on');};"
        " const release=(ev)=>{ev.preventDefault();el.classList.remove('active');sendCmd(key+'_off');};"
        " el.onpointerdown=press;"
        " el.onpointerup=release;"
        " el.onpointerleave=release;"
        "}"
        "bindHold('btn-up','up');"
        "bindHold('btn-down','down');"
        "bindHold('btn-left','left');"
        "bindHold('btn-right','right');"
        "for(let i=1;i<=8;i++){ bindHold('btn-f'+i,'f'+i); }"

        "/* ==== Panels under function buttons ==== */"
        "const panels=["
        " document.getElementById('panel-0'),"
        " document.getElementById('panel-1'),"
        " document.getElementById('panel-2'),"
        " document.getElementById('panel-3')"
        "];"

        "function refreshPanels(){"
        " fetch('/panels').then(r=>r.text()).then(t=>{"
        "  const lines=t.split('\\n');"
        "  for(let i=0;i<panels.length;i++){"
        "    if(!panels[i]) continue;"
        "    const line=(i<lines.length)?lines[i]:'';"
        "    panels[i].textContent=line||'\\u00A0';"
        "  }"
        " }).catch(()=>{});"
        "}"

        "const boxHist=document.getElementById('logbox-hist');"
        "function refreshHist(){"
        " fetch('/history').then(r=>r.text()).then(t=>{"
        "  const atBottom=boxHist.scrollTop+boxHist.clientHeight>=boxHist.scrollHeight-8;"
        "  boxHist.textContent=t;"
        "  if(atBottom) boxHist.scrollTop=boxHist.scrollHeight;"
        " }).catch(()=>{});"
        "}"

        "setInterval(()=>{refreshPanels();refreshHist();},500);"
        "refreshPanels();"
        "refreshHist();"

        "document.getElementById('clear-hist').onclick=()=>{"
        "  fetch('/clearHistory').then(()=>refreshHist());"
        "};"

        "</script>"
        "</body></html>";

    return html;
}

/*********************************************************************************************************
 * @brief   Handle Root
 * @details This function handles the root URL ("/") request and serves the HTML page.
 ********************************************************************************************************/
void ControlUI::handleRoot()
{
    server->send(200, "text/html", buildPage());
}

/*********************************************************************************************************
 * @brief   Handle Not Found
 * @details This function handles requests to unknown URLs by redirecting them to the root URL.
 ********************************************************************************************************/
void ControlUI::handleNotFound()
{
    handleRoot();
}

/*********************************************************************************************************
 * @brief   Handle Button
 * @details This function handles button press/release requests from the client. It updates the internal
 *          state of each button based on the received parameters.
 ********************************************************************************************************/
void ControlUI::handleBtn()
{
    if (!server->hasArg("b"))
    {
        server->send(400, "text/plain", "Missing param");
        return;
    }

    String b = server->arg("b");
    b.toLowerCase();

    // D-pad
    if (b == "up_on")
    {
        upHeld = true;
        upPressed = true;
    }
    else if (b == "up_off")
    {
        upHeld = false;
    }
    else if (b == "down_on")
    {
        downHeld = true;
        downPressed = true;
    }
    else if (b == "down_off")
    {
        downHeld = false;
    }
    else if (b == "left_on")
    {
        leftHeld = true;
        leftPressed = true;
    }
    else if (b == "left_off")
    {
        leftHeld = false;
    }
    else if (b == "right_on")
    {
        rightHeld = true;
        rightPressed = true;
    }
    else if (b == "right_off")
    {
        rightHeld = false;
    }

    // Function on/off
    else if (b == "f1_on")
    {
        funcHeld[0] = true;
        funcPressed[0] = true;
    }
    else if (b == "f1_off")
    {
        funcHeld[0] = false;
    }
    else if (b == "f2_on")
    {
        funcHeld[1] = true;
        funcPressed[1] = true;
    }
    else if (b == "f2_off")
    {
        funcHeld[1] = false;
    }
    else if (b == "f3_on")
    {
        funcHeld[2] = true;
        funcPressed[2] = true;
    }
    else if (b == "f3_off")
    {
        funcHeld[2] = false;
    }
    else if (b == "f4_on")
    {
        funcHeld[3] = true;
        funcPressed[3] = true;
    }
    else if (b == "f4_off")
    {
        funcHeld[3] = false;
    }
    else if (b == "f5_on")
    {
        funcHeld[4] = true;
        funcPressed[4] = true;
    }
    else if (b == "f5_off")
    {
        funcHeld[4] = false;
    }
    else if (b == "f6_on")
    {
        funcHeld[5] = true;
        funcPressed[5] = true;
    }
    else if (b == "f6_off")
    {
        funcHeld[5] = false;
    }
    else if (b == "f7_on")
    {
        funcHeld[6] = true;
        funcPressed[6] = true;
    }
    else if (b == "f7_off")
    {
        funcHeld[6] = false;
    }
    else if (b == "f8_on")
    {
        funcHeld[7] = true;
        funcPressed[7] = true;
    }
    else if (b == "f8_off")
    {
        funcHeld[7] = false;
    }
    else
    {
        server->send(400, "text/plain", "Unknown");
        return;
    }

    server->send(200, "text/plain", "OK");
}

/*********************************************************************************************************
 * @brief   Remote Begin
 * @details This function initializes the WiFi access point, DNS server, and HTTP server for remote
 *          control.
 ********************************************************************************************************/
void ControlUI::begin()
{
    if (!server)
        server = new WebServer(80);
    if (!dnsServer)
        dnsServer = new DNSServer();

    WiFi.mode(WIFI_AP);
    WiFi.softAP("NODDBALANCE_AP", "12345678");
    IPAddress apIP = WiFi.softAPIP();
    Serial.print("AP IP: ");
    Serial.println(apIP);

    dnsServer->start(DNS_PORT, "*", apIP);

    using std::placeholders::_1;

    server->on("/", std::bind(&ControlUI::handleRoot, this));
    server->on("/btn", std::bind(&ControlUI::handleBtn, this));

    server->on("/history", std::bind(&ControlUI::handleHistory, this));
    server->on("/clearHistory", std::bind(&ControlUI::handleClearHistory, this));
    server->on("/panels", std::bind(&ControlUI::handlePanels, this));

    server->on("/generate_204", std::bind(&ControlUI::handleRoot, this));
    server->on("/hotspot-detect.html", std::bind(&ControlUI::handleRoot, this));
    server->on("/ncsi.txt", std::bind(&ControlUI::handleRoot, this));
    server->onNotFound(std::bind(&ControlUI::handleNotFound, this));

    server->begin();
    Serial.println("HTTP server started");
}

/*********************************************************************************************************
 * @brief   Remote Process
 * @details This function processes incoming DNS and HTTP requests for remote control.
 ********************************************************************************************************/
void ControlUI::remoteProcess()
{
    if (dnsServer)
        dnsServer->processNextRequest();
    if (server)
        server->handleClient();
}

/*********************************************************************************************************
 * @brief   Format Time Display
 * @details This function formats a given time in milliseconds into a string representation of minutes,
 *          seconds, and milliseconds.
 ********************************************************************************************************/
String ControlUI::formatMillis_(uint32_t ms)
{
    uint32_t s = ms / 1000;
    uint32_t m = s / 60;
    uint32_t sec = s % 60;
    uint32_t ms3 = ms % 1000;
    char buf[16];
    snprintf(buf, sizeof(buf), "%02lu:%02lu.%03lu",
             (unsigned long)m, (unsigned long)sec, (unsigned long)ms3);
    return String(buf);
}

/*********************************************************************************************************
 * @brief   Push History
 * @details This function adds a new log entry to the history buffer, managing the circular buffer
 *          structure.
 ********************************************************************************************************/
void ControlUI::pushHist_(const String &line)
{
    uint16_t pos = (histStart + histSize) % HIST_CAP;
    histBuf[pos] = line;
    if (histSize < HIST_CAP)
        histSize++;
    else
        histStart = (histStart + 1) % HIST_CAP;
}

/*********************************************************************************************************
 * @brief   Log Append
 * @details This function appends a log entry to the history, adding a timestamp and handling multiple
 *          lines if necessary.
 ********************************************************************************************************/
void ControlUI::logAppend(const String &text)
{
    int start = 0;
    while (start < (int)text.length())
    {
        int nl = text.indexOf('\n', start);
        String line = (nl == -1) ? text.substring(start) : text.substring(start, nl);
        line.trim();
        if (line.length())
        {
            // thêm timestamp
            String stamp = "[" + formatMillis_(millis()) + "] " + line;
            pushHist_(stamp);
            Serial.println(stamp);
        }
        if (nl == -1)
            break;
        start = nl + 1;
    }
}

/*********************************************************************************************************
 * @brief   Clear History
 * @details This function clears the history buffer by resetting the start index and size.
 ********************************************************************************************************/
void ControlUI::clearHistory()
{
    histStart = 0;
    histSize = 0;
}

/*********************************************************************************************************
 * @brief   Build History
 * @details This function constructs a single string containing all log entries in the history buffer,
 *          separated by newlines.
 ********************************************************************************************************/
String ControlUI::buildHistory_()
{
    String out;
    out.reserve(4096);
    for (uint16_t i = 0; i < histSize; ++i)
    {
        uint16_t idx = (histStart + i) % HIST_CAP;
        out += histBuf[idx];
        out += '\n';
    }
    return out;
}

/*********************************************************************************************************
 * @brief   Handle History
 * @details This function handles the "/history" HTTP request and sends the log history to the client.
 ********************************************************************************************************/
void ControlUI::handleHistory()
{
    Serial.println("[handleHistory] called");
    server->send(200, "text/plain; charset=utf-8", buildHistory_());
}

/*********************************************************************************************************
 * @brief   Handle Clear History
 * @details This function handles the "/clearHistory" HTTP request and clears the log history.
 ********************************************************************************************************/
void ControlUI::handleClearHistory()
{
    clearHistory();
    server->send(200, "text/plain", "CLEARED");
}

/*********************************************************************************************************
 * @brief   Add Log to Panel
 * @details This function adds a log entry to a specified panel position.
 ********************************************************************************************************/
void ControlUI::addLog(const String &text, uint8_t position)
{
    if (position >= PANEL_COUNT)
        return;
    panelText[position] = text;
}

/*********************************************************************************************************
 * @brief   Build Panels
 * @details This function constructs a single string containing all panel texts, separated by newlines.
 ********************************************************************************************************/
String ControlUI::buildPanels_()
{
    String out;
    out.reserve(256);
    for (uint8_t i = 0; i < PANEL_COUNT; ++i)
    {
        out += panelText[i];
        out += '\n';
    }
    return out;
}

/*********************************************************************************************************
 * @brief   Handle Panels
 * @details This function handles the "/panels" HTTP request and sends the panel texts to the client.
 ********************************************************************************************************/
void ControlUI::handlePanels()
{
    server->send(200, "text/plain; charset=utf-8", buildPanels_());
}