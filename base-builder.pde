import processing.serial.*;

Serial UART;
PrintWriter datafile;
String data_series;

void setup() {
  UART = new Serial(this, "COM3", 115200);  // The port is COM3
  datafile = createWriter("AA.txt");
}

void draw ()
{
  if (data_series != null) {
    datafile.print(data_series);
  }
}

void serialEvent (Serial UART) { 
   String data_received = UART.readStringUntil('\n');
   if (data_received != null)
   {
     data_series = data_received;
   }
}

void keyPressed()
{
  datafile.flush();
  datafile.close();
  exit();
}
