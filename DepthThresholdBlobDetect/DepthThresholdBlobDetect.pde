import blobDetection.*;
import org.openkinect.processing.*;

// What features to turn on?
boolean closestPoint = false;
boolean avgPoint = false;
boolean blobDetect = true;

Kinect2 kinect2;
BlobDetection blobs;

// Depth image
PImage depthImg;

// Which pixels do we care about?
float minDepth =  1;
float maxDepth =  1000; // .75m
float blobThreshold = 0.5;

// What is the kinect's angle
float angle;

void setup() {
  size(1024, 848);

  kinect2 = new Kinect2(this);
  kinect2.initDepth();
  kinect2.initDevice();
  
  if (blobDetect) {
    blobs = new BlobDetection(kinect2.depthWidth, kinect2.depthHeight);
    blobs.setPosDiscrimination(true);
    blobs.setThreshold(blobThreshold);
  }

  // Blank image
  depthImg = new PImage(kinect2.depthWidth, kinect2.depthHeight);
}

void draw() {
  // Update threshold based on mouse position (for calibration)
  // minDepth = map(mouseX, 0, width, 0, 4500);
  // maxDepth = map(mouseY, 0, height, 0, 4500);
  // float blobThreshold = map(mouseX, 0, width, 0.0, 1.0);
  // blobs.setThreshold(blobThreshold);

  // Draw the raw image
  PImage img = kinect2.getDepthImage();
  
  image(img, 0, 0, width, height);

  // Threshold the depth image
  int[] rawDepth = kinect2.getRawDepth();
  
  boolean detected = false;
  
  float sumX = 0;
  float sumY = 0;
  int pixCount = 0;
  
  int closest = 4501;
  int closestX = 0;
  int closestY = 0;
  
  for (int x=0; x < kinect2.depthWidth; x++) {
    for (int y=0; y < kinect2.depthHeight; y++) {
      int offset = x + y * kinect2.depthWidth;
      int d = rawDepth[offset];
      
      /*
      if (d == 2048) {
        d = 4500;
      }
      */
      
      if (d >= minDepth && d <= maxDepth) { // && x > 50 && y > 50 && x < kinect2.depthWidth - 50 && y < kinect2.depthHeight - 50) {
        depthImg.pixels[offset] = color(map(d, minDepth, maxDepth, 255, 129), 128, 255);
        
        pixCount++;
        if (avgPoint) {
          sumX += x;
          sumY += y;
        }
        
        if (closestPoint) {
          if (d < closest) {
            closest = d;
            closestX = x;
            closestY = y;
          }
        }
        
        if (pixCount >= 200) {
          detected = true;
        } else {
          detected = false;
        }
      } else {
        depthImg.pixels[offset] = 0; //img.pixels[offset];
      }
    }
  }
  
  // Draw the thresholded image
  depthImg.updatePixels();
  //image(depthImg, kinect2.depthWidth, 0);
  
  image(depthImg, 0, 0, width, height);
  
  fill(255, 0, 300);
  text("TILT: " + angle, 10, 20);
  text("THRESHOLD: [" + minDepth + ", " + maxDepth + "]", 10, 50);
  if (blobDetect) {
    text("BLOB THRESHOLD: " + blobThreshold, 10, 80);
  }
  
  if (detected) {
    textSize(20);
    text("*** FOUND SOMETHING! ***", 10, 140);
    
    if (avgPoint) {
      float avgX = sumX / pixCount;
      float avgY = sumY / pixCount;
      
      fill(150, 0, 255);
      ellipse(map(avgX, 0, kinect2.depthWidth, 0, width), map(avgY, 0, kinect2.depthHeight, 0, height), 32, 32);
    }
    
    if (closestPoint) {
      fill(0, 150, 255);
      stroke(0, 150, 255);
      strokeWeight(2);
      ellipse(map(closestX, 0, kinect2.depthWidth, 0, width), map(closestY, 0, kinect2.depthHeight, 0, height), 32, 32);
      text("CLOSEST POINT: [" + closestX + ", " + closestY + "]" + " = " + closest * 0.001 + " meters", 10, 110);
    }
    
    if (blobDetect) {
      blobs.computeBlobs(depthImg.pixels);
      fill(150, 255, 255);
      int nBlobs = blobs.getBlobNb();
      int sigBlobs = 0;
      for (int n = 0; n < nBlobs; n++) {
        Blob b = blobs.getBlob(n);
        fill(150, 255, 255);
        float bx = map(b.x, 0, 1, 0, width);
        float by = map(b.y, 0, 1, 0, height);
        float bxmax = map(b.xMax, 0, 1, 0, width);
        float bymax = map(b.yMax, 0, 1, 0, height);
        float bxmin = map(b.xMin, 0, 1, 0, width);
        float bymin = map(b.yMin, 0, 1, 0, height);
        float bwidth = bxmax-bxmin;
        float bheight = bymax-bymin;
        float barea = bwidth * bheight;
        if (barea > 10000 && bwidth > 80 && bheight > 80) {
          sigBlobs++;
          ellipse(bx, by, 18, 18);
        }
      }
      text("SIGNIFICANT BLOBS FOUND: " + sigBlobs + " | TOTAL BLOBS FOUND: " + nBlobs, 10, 170);
    }
  }
}