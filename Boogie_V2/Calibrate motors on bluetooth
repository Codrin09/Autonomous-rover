if(btSerial.available()){
String incomingData = btSerial.readString();
switch (incomingData.toInt()){
    case 2:
    btSerial.println("Scaling motor 0 speed by: " + String(++speedScalar[0]));
    break;
    case 3:
    btSerial.println("Scaling motor 0 speed by: " + String(--speedScalar[0]));
    break;
    case 4:
    btSerial.println("Scaling motor 1 speed by: " + String(++speedScalar[1]));
    break;
    case 5:
    btSerial.println("Scaling motor 1 speed by " + String(--speedScalar[1]));
    break;
    default:
    btSerial.println("No valid command received");
    break;
}
}