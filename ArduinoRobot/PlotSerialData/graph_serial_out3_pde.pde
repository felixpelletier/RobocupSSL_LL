// Graphing sketch


import processing.serial.*;

Serial myPort;		  // The serial port
ArrayList<FloatList> datas = new ArrayList<FloatList>();

float scalling = 1;
float bitches = 0;

// Graphics width
int w = 900;

//  debug
boolean doSerial = true;


void setup () {
  // set the window size:
  size(w + 100, 600);		  

  // List all the available serial ports
  println(Serial.list());
  if (doSerial) {
    myPort = new Serial(this, Serial.list()[0], 115200);
    myPort.bufferUntil('\n');
  }

  // set inital background:
  background(#0B1432);
}
void draw () {
  //noLoop();
  background(#0B1432);
  
  // Draw each variables
  for(int i = 0; i < datas.size(); i++){
    //drawGraph(i + 1, datas.size(), datas.get(i).min(), datas.get(i).max());
  }
  
  if(datas.size() > 0){
    text("Data:", w, 100); 
    text(datas.get(0).size(), w + 35, 100);
    text("Data/s:", w, 140); 
    text((float)(datas.get(0).size())/millis()*1000, w + 50, 140);
    //text("Scale:", w, 160); 
    text(bitches, w, 160);
    //text(datas.get(0).max(), w, 160); //32752
    //text(scalling, w + 30, 160);
  }
}

void drawGraph(int id, float nbrG, float min, float max) {
  // Set color
  stroke(#E6CC94);
  fill(#E6CC94);

  // Set the start and end id
  int start, end;
  if (scalling * datas.get(id-1).size() < w) {
    start = 0;
    end = datas.get(0).size();
  }
  else {
    start = (int)(datas.get(id-1).size()-(float)(w)/scalling);
    end = datas.get(0).size();
  }

  beginShape();
  vertex(0, (min)/(max-min)*height/nbrG + height*(id/nbrG));
  for (int i = start + 1; i < end;i++) { 
    // We call the current point and the last one
    float preByte = map(datas.get(id-1).get(i - 1), min, max, 0, height/nbrG);
    float inByte = map(datas.get(id-1).get(i), min, max, 0, height/nbrG);
    
    // We draw a line between the previous point and the new one
    vertex((float)(i - 1 - start)*scalling, height*(id/nbrG) - preByte);
    vertex((float)(i - start)*scalling, height*(id/nbrG) - inByte);
  }
  vertex((datas.get(id-1).size() - 1 - start)*scalling, (min)/(max-min)*height/nbrG +height*(id/nbrG) - 1);
  endShape();
}

void serialEvent (Serial myPort) {
  String inString = myPort.readStringUntil('\n');
  println(inString);

  if (inString != null && !inString.equals("")) {
    //inString.replace("\n", "");
    String[] data = split(inString, " "); //sépare le nom de la valeur recue
    //println(data);
    if(data.length > datas.size()){
      for(int i = datas.size(); i < data.length; i++){
        datas.add(new FloatList());
      }
    }
    
    for(int i = 0; i < data.length; i++){
      bitches += (float(data[i]) - 185)*0.00001875;
      //bitches += (float(data[i]) - 55)*0.00001775;
      //datas.get(i).append(float(data[i]));
    }
  }
}
void mouseClicked() {
  //redraw();
  bitches = 0;
  if (mouseButton == LEFT)
    scalling*=2;
  else
    scalling/=2;
}

// debug
void mouseMoved() {
  if (!doSerial) {
    float inByte = float(mouseX) - width/2; 
    float inByte2 = float(mouseY) -height/2; 
    String inString = inByte + " " + inByte2;
    String[] data = split(inString, " ");
    
    // We add the news columns of data
    if(data.length > datas.size()){
      for(int i = datas.size(); i < data.length; i++){
        datas.add(new FloatList());
      }
    }
    
    for(int i = 0; i < data.length; i++){
      datas.get(i).append(float(data[i]));
    }
  }
}

void keyPressed() {
  if(key == 's'){
    print(millis());
    ArrayList<FloatList> d = datas;
    
    // Filename format : out_YYYY-MM-DD_hh:mm:ss.cvs
    PrintWriter output = createWriter("out_"+String.valueOf(year())+"-"+String.valueOf(month())+"-"+String.valueOf(day())+"_"+String.valueOf(hour())+"-"+String.valueOf(minute())+"-"+String.valueOf(second())+"-"+String.valueOf(millis())+".cvs"); 
  
    for(int j = 0; j < d.get(0).size(); j++){
       output.print(j+",");
      for(int i = 0; i < d.size() - 1; i++){
        output.print(d.get(i).get(j)+",");
      }
      output.println(d.get(d.size() - 1).get(j));
    } 
    
    output.flush();
    output.close();
    print("File saved!");
  }
}


