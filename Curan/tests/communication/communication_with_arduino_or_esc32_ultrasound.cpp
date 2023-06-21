#include <asio.hpp>
#include <iostream>

char code_corresponding_in_arduino[] = R"(
// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
int distance;
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  if (Serial.available() > 0) {
    while(Serial.available() > 0) {
      char t = Serial.read();
    }
    Serial.print(distance);
    Serial.print('e');
  }    
  delay(100);
}
})";

constexpr size_t maximum_length_of_message = 4; 

int main(int argc, char* argv[]){
try{
    std::string serial_connection_name = std::string(CURAN_SERIAL_PORT);
    if(serial_connection_name.size()==0){
        if(argc<2){
            std::cout << "To use this service please provide the port number of the serial connection \neg. windows will be COM ports, linux will use /dev/ttyS* or /dev/ttyUSB*, etc\n" ;
            return 1;
        }
        serial_connection_name = std::string(argv[1]);
    }
    asio::io_context context;
    asio::serial_port serial(context);
    serial.open(serial_connection_name);

    std::atomic<bool> value = true;
    auto stopper = [&value,&serial](){
        std::string input;
        std::cin >> input;
        value.store(false);
        serial.close();
    };
    std::thread to_stop{stopper};

 
    char to_send = 10;
    size_t nread = 0;
    for (int counter = 0;value.load();++counter) {
        asio::write(serial,asio::buffer(&to_send,1));
        asio::streambuf input_buffer;
        nread = asio::read_until(
            serial, input_buffer, 'e'
        );
        std::istream is(&input_buffer);
        std::string line;
        std::getline(is, line);
        std::cout << "Received: (" << line << ") cm pressed: " << counter << "\n";
    }
    to_stop.join();
    return 0; 
} catch(std::exception & e){
    std::cout << "exception thrown while reading the serial stream\n";
    return 1;
}

}