// #include <Adafruit_NeoPixel.h>
// #include <settings.h>
// // NeoPixel setup
// #define DATA_PIN 12
// #define POWER_PIN 11
// #define NUMPIXELS 1
//
// Adafruit_NeoPixel pixels(NUMPIXELS, DATA_PIN, NEO_GRB + NEO_KHZ800);
//
// void setupNeoPixel();
// void setNeoPixelColor(String colorName);
//
// void setNeoPixelColor(String colorName) {
//     uint32_t color;
//     if (colorName == "red") {
//         color = pixels.Color(255, 0, 0);
//     } else if (colorName == "green") {
//         color = pixels.Color(0, 255, 0);
//     } else if (colorName == "blue") {
//         color = pixels.Color(0, 0, 255);
//     } else if (colorName == "yellow") {
//         color = pixels.Color(255, 255, 0);
//     } else if (colorName == "cyan") {
//         color = pixels.Color(0, 255, 255);
//     } else if (colorName == "magenta") {
//         color = pixels.Color(255, 0, 255);
//     } else if (colorName == "white") {
//         color = pixels.Color(255, 255, 255);
//     } else if (colorName == "orange") {
//         color = pixels.Color(255, 165, 0);
//     } else if (colorName == "purple") {
//         color = pixels.Color(128, 0, 128);
//     } else {
//         color = pixels.Color(0, 0, 0); // default to off if color name is not recognized
//     }
//     pixels.setPixelColor(0, color);
//     pixels.show();
// }
//
// void setupNeoPixel() {
//     pinMode(POWER_PIN, OUTPUT);
//     digitalWrite(POWER_PIN, HIGH);
//     pixels.begin();
//     pixels.clear();
// }
//