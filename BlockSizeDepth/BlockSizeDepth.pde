import org.openkinect.processing.*;

Kinect2 kinect2;

// Connect the Kinect2 and explore using depth image data

void setup() {
  size(1024, 848, P3D);
  kinect2 = new Kinect2(this);
  
  kinect2.initDepth();
  kinect2.initDevice();
}

void draw() {
  background(0);
  
  PImage img = kinect2.getDepthImage();
  image(img, 0, 0, width, height);
  
  int skip = 16;
  for (int x = 0; x < img.width; x+=skip) {
    for (int y = 0; y < img.height; y+=skip) {
      int index = x + y * img.width;
      float b = brightness(img.pixels[index]);
      float z = map(b, 0, 255, 300, -300);
      fill(255-b);
      pushMatrix();
      translate(2*x,2*y,z);
      rect(0, 0, skip/2, skip/2);
      popMatrix();
    }
  }
}