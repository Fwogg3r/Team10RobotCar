
// without obstacle avoidance 
// https://github.com/Jeydori/Line-Follower-with-and-without-Obstacle-Avoidance-on-Specific-Track-?tab=readme-ov-file
#define enA 3
        #define in1 4 
        #define in2 5 
        #define in3 6  
        #define in4 7 
        #define enB 9 
        
        #define L_S A0 
        #define R_S A1 
        
        void setup(){ 
        pinMode(R_S, INPUT); 
        pinMode(L_S, INPUT); 
        
        pinMode(enA, OUTPUT);  
        pinMode(in1, OUTPUT); 
        pinMode(in2, OUTPUT); 
        pinMode(in3, OUTPUT);   
        pinMode(in4, OUTPUT);  
        pinMode(enB, OUTPUT); 
        
        //adjust depending on the IR sensor's response time and calibrate it properly
        analogWrite(enA, 210); 
        analogWrite(enB, 210); 
        
        delay(500);
        }
        
        void loop(){
        //if Right Sensor and Left Sensor are at White color then it will call forword function
         if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
         forward(); 
         }
        //if Right Sensor is Black and Left Sensor is White then it will call turn Right function
        else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){
         turnRight();
         }
        //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
        else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){
         turnLeft();
         } 
        }
        
        void forward(){  //forword
        digitalWrite(in1, LOW); 
        digitalWrite(in2, HIGH); 
        digitalWrite(in3, HIGH); 
        digitalWrite(in4, LOW); 
        }
        
        void backword(){ //backword
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW);  
        digitalWrite(in3, LOW); 
        digitalWrite(in4, HIGH); 
        }
        
        void turnRight(){ //turnRight
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW); 
        digitalWrite(in3, HIGH);  
        digitalWrite(in4, LOW); 
        }
        
        void turnLeft(){ //turnLeft
        digitalWrite(in1, LOW);  
        digitalWrite(in2, HIGH);  
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH); 
        }
        
        void Stop(){ //stop
        digitalWrite(in1, LOW); 
        digitalWrite(in2, LOW); 
        digitalWrite(in3, LOW);  
        digitalWrite(in4, LOW);
        }
