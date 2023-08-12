
/*  https://github.com/sb-ocr/diy-spacemouse  */

#include <Arduino.h>
#include <Tlv493d.h>
// #include <TinyUSB_Mouse_and_Keyboard.h>
#include <Adafruit_TinyUSB.h>
#include "FastLED.h"
#include <OneButton.h>
#include <SimpleKalmanFilter.h>

#define NUM_LEDS 3
#define DATA_PIN 3
#define QWIIC_I2C_PORT Wire1

#define COLOR_UNCALIB (CRGB::Red)
#define COLOR_CALIBDONE (CRGB::Blue)
#define COLOR_PAN (CRGB::Green)
#define COLOR_ORBIT (CRGB::Orange)
#define COLOR_RAINBOW_POS0 (CRGB::Red)
#define COLOR_RAINBOW_POS1 (CRGB::Green)
#define COLOR_RAINBOW_POS2 (CRGB::Blue)

// Report ID
enum
{
  RID_KEYBOARD = 1,
  RID_MOUSE
};

//  HID report descriptor using TinyUSB's template
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(RID_MOUSE))};
// USB HID object.
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report),
                          HID_ITF_PROTOCOL_NONE, 4, false);

// Tlv493d Opject
// Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();
Tlv493d mag = Tlv493d();
CRGB leds[NUM_LEDS];
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);
// void pride(void);

// Setup buttons
OneButton button1(27, true);
OneButton button2(24, true);

float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;

int calSamples = 300;
// int sensivity = 8; //org
int sensivity = 16;
int magRange = 3;
int outRange = 127;      // Max allowed in HID report
float xyThreshold = 0.4; // Center threshold

int inRange = magRange * sensivity;
float zThreshold = xyThreshold * 1.5;

bool isOrbit = false;
bool mouseMoving = false;

uint8_t gHue = 0;
uint16_t loopnum = 0;
uint8_t keycode[6] = {0};
uint8_t modifier = 0;

void nblendU8TowardU8(uint8_t &cur, const uint8_t target, uint8_t amount);
CRGB fadeTowardColor(CRGB &cur, const CRGB &target, uint8_t amount);
void fadeTowardColor(CRGB *L, uint16_t N, const CRGB &bgColor, uint8_t fadeAmount);
void goHome(void);
void fitToScreen(void);

void setup()
{
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // GRB ordering is assumed

  FastLED.setBrightness(24);

  button1.attachClick(goHome);
  button1.attachLongPressStop(goHome);

  button2.attachClick(fitToScreen);
  button2.attachLongPressStop(fitToScreen);
  Serial.begin(115200);
  // while (!Serial)
  //   ;

  usb_hid.setStringDescriptor("TinyUSB HID Composite");

  usb_hid.begin();
  // wait until device mounted
  while (!TinyUSBDevice.mounted())
  {
    delay(1);
  }
  fill_solid(leds, NUM_LEDS, COLOR_UNCALIB);
  FastLED.show();
  mag.begin(QWIIC_I2C_PORT, TLV493D_ADDRESS2, true);
  mag.setAccessMode(mag.MASTERCONTROLLEDMODE);
  mag.disableTemp();
  // crude offset calibration on first boot
  for (int i = 1; i <= calSamples; i++)
  {
    delay(mag.getMeasurementDelay());
    mag.updateData();

    xOffset += mag.getX();
    yOffset += mag.getY();
    zOffset += mag.getZ();

    Serial.print(".");
    fadeTowardColor(leds, NUM_LEDS, COLOR_CALIBDONE, 1);
    FastLED.show();
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println();
  Serial.println(xOffset);
  Serial.println(yOffset);
  Serial.println(zOffset);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  delay(100);
  FastLED.setBrightness(12);
  leds[0] = COLOR_RAINBOW_POS0;
  leds[1] = COLOR_RAINBOW_POS1;
  leds[2] = COLOR_RAINBOW_POS2;
  FastLED.show();
  // mouse and keyboard init
  // Mouse.begin();
  // Keyboard.begin();

  Serial.println("TinyUSB HID mouse key Composite");
}

void loop()
{

  // keep watching the push buttons
  button1.tick();
  button2.tick();

  // get the mag data
  delay(mag.getMeasurementDelay());
  mag.updateData();

  // update the filters
  xCurrent = xFilter.updateEstimate(mag.getX() - xOffset);
  yCurrent = yFilter.updateEstimate(mag.getY() - yOffset);
  zCurrent = zFilter.updateEstimate(mag.getZ() - zOffset);

  // check the center threshold
  if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold)
  {

    int xMove = 0;
    int yMove = 0;

    // map the magnetometer xy to the allowed 127 range in HID repports
    xMove = map(xCurrent, -inRange, inRange, -outRange, outRange);
    yMove = map(yCurrent, -inRange, inRange, -outRange, outRange);
    // Remote wakeup
    if (TinyUSBDevice.suspended())
    {
      // Wake up host if we are in suspend mode
      // and REMOTE_WAKEUP feature is enabled by host
      TinyUSBDevice.remoteWakeup();
    }
    // press shift to orbit in Fusion 360 if the pan threshold is not corssed (zAxis)
    if (abs(zCurrent) < zThreshold && !isOrbit)
    {
      // Keyboard.press(KEY_LEFT_SHIFT);
      if (usb_hid.ready() && isOrbit == false)
      {
        modifier = KEYBOARD_MODIFIER_LEFTSHIFT;
        usb_hid.keyboardReport(RID_KEYBOARD, modifier, keycode);
        delay(10);
        isOrbit = true;
      }
    }

    // pan or orbit by holding the middle mouse button and moving propotionaly to the xy axis
    // Mouse.press(MOUSE_MIDDLE);
    if (usb_hid.ready())
    {
      mouseMoving = true;
      // Mouse.move(yMove, xMove, 0);
      usb_hid.mouseReport(RID_MOUSE, MOUSE_BUTTON_MIDDLE, xMove, yMove, 0, 0);
      delay(10);
      // for seisitivity monitor
      // Serial.print(xMove);
      // Serial.print(",");
      // Serial.print(yMove);
      // Serial.print(",");
      // Serial.print(zCurrent);
      // Serial.println();
    }
  }
  else
  {

    // release the mouse and keyboard if within the center threshold
    // Mouse.release(MOUSE_MIDDLE);
    if (usb_hid.ready())
    {
      usb_hid.mouseButtonRelease(RID_MOUSE);
      delay(10);
      // Keyboard.releaseAll();
      usb_hid.keyboardRelease(RID_KEYBOARD);
      delay(10);
    }
    if (mouseMoving)
    {
      mouseMoving = false;
      leds[0] = COLOR_RAINBOW_POS0;
      leds[1] = COLOR_RAINBOW_POS1;
      leds[2] = COLOR_RAINBOW_POS2;
      FastLED.show();
    }
    isOrbit = false;
    modifier = 0;
  }

  Serial.print(xCurrent);
  Serial.print(",");
  Serial.print(yCurrent);
  Serial.print(",");
  Serial.print(zCurrent);
  Serial.println();
  if (mouseMoving)
  {
    if (isOrbit)
    {
      fill_solid(leds, NUM_LEDS, COLOR_ORBIT);
    }
    else
    {
      fill_solid(leds, NUM_LEDS, COLOR_PAN);
    }
    FastLED.show();
  }
  else
  {
    loopnum++;
    uint8_t gHue_temp = (uint8_t)(loopnum >> 2);
    if (gHue != gHue_temp)
    {
      gHue = gHue_temp;
      fill_rainbow_circular(leds, NUM_LEDS, gHue, false);
      FastLED.show();
    }
  }
}

//
// go to home view in Fusion 360 by pressing  (CTRL + SHIFT + H) shortcut assigned to the custom Add-in command
void goHome()
{

  if (TinyUSBDevice.suspended())
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    TinyUSBDevice.remoteWakeup();
  }
  // Keyboard.press(KEY_LEFT_GUI);
  // Keyboard.press(KEY_LEFT_SHIFT);
  // Keyboard.write('h');
  if (usb_hid.ready())
  {

    keycode[0] = HID_KEY_H;
    // for win & mac both enable combination
    modifier = KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTCTRL;
    usb_hid.keyboardReport(RID_KEYBOARD, modifier, keycode);
    delay(10);
    // Keyboard.releaseAll();
    usb_hid.keyboardRelease(RID_KEYBOARD);
    modifier = 0;
    delay(10);
  }
  Serial.println("pressed home");
}

// fit to view by pressing the middle mouse button twice
void fitToScreen()
{
  if (TinyUSBDevice.suspended())
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    TinyUSBDevice.remoteWakeup();
  }
  // Mouse.press(MOUSE_MIDDLE);
  if (usb_hid.ready())
  {
    usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
    delay(10);
    // Mouse.release(MOUSE_MIDDLE);
    usb_hid.mouseButtonRelease(RID_MOUSE);
    delay(10);
    // Mouse.press(MOUSE_MIDDLE);
    usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
    delay(10);
    // Mouse.release(MOUSE_MIDDLE);
    usb_hid.mouseButtonRelease(RID_MOUSE);
    delay(10);
  }
  Serial.println("pressed fit");
}

// Helper function that blends one uint8_t toward another by a given amount
void nblendU8TowardU8(uint8_t &cur, const uint8_t target, uint8_t amount)
{
  if (cur == target)
    return;

  if (cur < target)
  {
    uint8_t delta = target - cur;
    delta = scale8_video(delta, amount);
    cur += delta;
  }
  else
  {
    uint8_t delta = cur - target;
    delta = scale8_video(delta, amount);
    cur -= delta;
  }
}

// Blend one CRGB color toward another CRGB color by a given amount.
// Blending is linear, and done in the RGB color space.
// This function modifies 'cur' in place.
CRGB fadeTowardColor(CRGB &cur, const CRGB &target, uint8_t amount)
{
  nblendU8TowardU8(cur.red, target.red, amount);
  nblendU8TowardU8(cur.green, target.green, amount);
  nblendU8TowardU8(cur.blue, target.blue, amount);
  return cur;
}

// Fade an entire array of CRGBs toward a given background color by a given amount
// This function modifies the pixel array in place.
void fadeTowardColor(CRGB *L, uint16_t N, const CRGB &bgColor, uint8_t fadeAmount)
{
  for (uint16_t i = 0; i < N; i++)
  {
    fadeTowardColor(L[i], bgColor, fadeAmount);
  }
}
