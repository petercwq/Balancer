void setup()
{
    // initialize serial:
    Serial.begin(115200);
}

void loop()
{
    while (Serial.available())
    {
        Serial.print(Serial.read(), DEC);
    }
}
