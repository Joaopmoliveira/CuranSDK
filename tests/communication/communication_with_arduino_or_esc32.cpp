#include <asio.hpp>
#include <iostream>

char code_corresponding_in_arduino[] = R"(
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;  // the number of the pushbutton pin
const int ledPin = 13;    // the number of the LED pin

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status
int previousbuttonState = buttonState;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    if(previousbuttonState!=buttonState){
        if (Serial.available() > 0) {
          int incomingByte = Serial.read();
          Serial.write("up");
        }     
    }
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
  previousbuttonState = buttonState;
  delay(100);
}
})";

constexpr size_t maximum_length_of_message = 1000; 

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
        std::cout << "Enter Message: ";
        std::cin >> input;
        value.store(false);
        serial.close();
    };
    std::thread to_stop{stopper};

    char data[maximum_length_of_message];
    char byte = 10;
    size_t nread = 0;
    for (int counter = 0;value.load();++counter) {
        asio::write(serial,asio::buffer(&byte,1));
        nread = asio::read(
            serial, asio::buffer(data, 2)
        );
        std::string message(data, nread);
        std::cout << "Received: (";
        std::cout << message << ") pressed: " << counter << "\n";
    }
    to_stop.join();
    return 0; 
} catch(std::exception & e){
    std::cout << "exception thrown while reading the serial stream\n";
    return 1;
}

}