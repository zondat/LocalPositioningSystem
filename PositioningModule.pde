// Serial communication
import processing.serial.*;
String serialPortName = "COM14";
Serial serialPort; // Serial port object

public class Position {
  private int x=0;
  private int y=0;
  
  public Position(){};
  public Position(int x, int y) {
    this.x = x;
    this.y = y;
  }
  
  public int getX() {return x;}
  public int getY() {return y;}
};

// Visualization
static final int gridSize = 100; // The size of each square in the grid
static final int gridWidth = 100; // The width of the grid in squares
static final int gridHeight = 100; // The height of the grid in squares

void drawGridMap() {
  // Draw the grid
  //stroke(0);
  fill(255,233,140);
  for (int x = 0; x < gridWidth; x++) {
    for (int y = 0; y < gridHeight; y++) {
      rect(x * gridSize, y * gridSize, gridSize, gridSize);
    }
  }
}

void drawObject(Position pos) {
    // Draw the object
  fill(127, 255, 0);
  rect(pos.getX() * gridSize, pos.getY() * gridSize, gridSize, gridSize);  
}

void drawBase(Position basePosition, int r, int g, int b) {
  fill(r, g, b);
  rect(basePosition.getX() * gridSize, basePosition.getY() * gridSize, gridSize, gridSize);  
}



byte[] inBuffer = new byte[100]; // holds serial message
Position getPositionUpdate() {
  if (serialPort.available() > 0) {
     try {
        serialPort.readBytesUntil('\n', inBuffer);
        String mess = new String(inBuffer);
        println(mess);
        String[] coor = split(mess, ' ');
        int x = (int)(Integer.parseInt(coor[0].trim()));
        int y = (int)(Integer.parseInt(coor[1].trim()));
        return new Position(x/gridSize, y/gridSize);
      }
      catch (Exception e) {
        return null;
      }
  }
  return null;
}

void setup() {
  size(1000, 1000);
  drawGridMap();
  Position base1 = new Position(0,0);
  Position base2 = new Position(9, 0);
  Position base3 = new Position(0, 9);
  
  drawBase(base1, 0, 0, 255);
  drawBase(base2,255, 0, 0);
  drawBase(base3, 0, 255, 0);
  
  serialPort = new Serial(this, serialPortName, 9600);
}

void draw() {
  Position currentPosition = getPositionUpdate();
  if (currentPosition!=null) {
    drawObject(currentPosition);
  }
}

//void keyPressed() {
//  // Move the object based on the arrow keys
//  if (keyCode == UP) {
//    if (objectY > 0) {
//      objectY--;
//    }
//  } else if (keyCode == DOWN) {
//    if (objectY < gridHeight - 1) {
//      objectY++;
//    }
//  } else if (keyCode == LEFT) {
//    if (objectX > 0) {
//      objectX--;
//    }
//  } else if (keyCode == RIGHT) {
//    if (objectX < gridWidth - 1) {
//      objectX++;
//    }
//  }
//}
