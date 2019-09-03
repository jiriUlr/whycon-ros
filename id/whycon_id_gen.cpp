#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <bitset>
#include <Magick++.h>
#include "CNecklace.h"

using namespace Magick;
using namespace std;

// Define default parameters
bool verbose = false;
bool legacy = false;
int hamming = 1;
int xc = 900;
int yc = 900;
int index = 0;
int n;
float w;
Image whycodeTorso;

// Display help
void display_help(){
    printf("\nUsage: whycon-id-gen [options] bits\n");
    printf("    -h,    Display this help\n");
    printf("    -v,    Verbose while generating canvas (default: false)\n");
    printf("    -l,    Generating the original WhyCon marker\n");
    printf("    -d,    Set minimal Hamming distance (default: 1)\n\n");
}

// Draw the original WhyCon marker
Image draw_whycon_markers(){
    list<Drawable> drawList;
    Image image("1800x1800", "white");
    image.resolutionUnits(PixelsPerCentimeterResolution);
    image.density("100x100");

    // Generate original WhyCon marker
    drawList.push_back(DrawableStrokeColor("black"));
    drawList.push_back(DrawableFillColor("white"));
    drawList.push_back(DrawableEllipse(xc, yc, 899, 899, 0, 360));
    image.draw(drawList);
    drawList.clear();

    drawList.push_back(DrawableStrokeWidth(0));
    drawList.push_back(DrawableFillColor("black"));
    drawList.push_back(DrawableEllipse(xc, yc, 700, 700, 0, 360));
    image.draw(drawList);
    drawList.clear();

    drawList.push_back(DrawableStrokeWidth(0));
    drawList.push_back(DrawableFillColor("white"));
    drawList.push_back(DrawableEllipse(xc, yc, 420, 420, 0, 360));
    image.draw(drawList);
    drawList.clear();

    return image;
}

// Draw the encded ID into WhyCon marker
void draw_whycode_markers(int id, int idx, const int teethCount){
    list<Drawable> drawList;
    list<Coordinate> coordsList;

    if(verbose) printf("Generating WhyCode Canvas for Id %d (encoding %d)\n", idx, id);
//    Image image = draw_whycon_markers();
    Image image = whycodeTorso;

    // Convert lowest bit shift to binary
    if(verbose) printf("Converting ID to binary\n");
    string s = bitset<32>(id).to_string();

    drawList.push_back(DrawableStrokeWidth(0));
    drawList.push_back(DrawableFillColor("black"));

    // For each encoding bit
    double x1, y1, x2, y2;
    for(int i = 0; i < teethCount; i++){
        // Calculate the pixel positions of each segment
        x1 = xc + 650 * cos(-w * (2 * i + (s.at(i + 32 - teethCount) - '0') * 2.0) / 180.0 * M_PI);
        y1 = yc + 650 * sin(-w * (2 * i + (s.at(i + 32 - teethCount) - '0') * 2.0) / 180.0 * M_PI);
        x2 = xc + 650 * cos(-w * (2 * i + 1) / 180.0 * M_PI);
        y2 = yc + 650 * sin(-w * (2 * i + 1) / 180.0 * M_PI);

        list<Coordinate> coordsList;
        coordsList.push_back(Coordinate(xc, yc));
        coordsList.push_back(Coordinate(x1, y1));
        coordsList.push_back(Coordinate(x2, y2));

        // Draw each of the segments onto the original WhyCon marker
        if(verbose) printf("Drawing Segment Size: %f %f %f %f\n", x1 ,y1 ,x2, y2);
        drawList.push_back(DrawablePolygon(coordsList));
        coordsList.clear();
    }
    image.draw(drawList);
    drawList.clear();

    // Draw a final white circle in the centre to complete the marker
    printf("Rendering final image: %d (encoding %d)  =>  %08d.png\n", idx, id, idx);
    drawList.push_back(DrawableStrokeWidth(0));
    drawList.push_back(DrawableFillColor("white"));
    drawList.push_back(DrawableEllipse(xc, yc, 240, 240, 0, 360));
    image.draw(drawList);
    drawList.clear();

    char infoText[13];
    sprintf(infoText, "ID: %d", idx);
    drawList.push_back(DrawableText(1600, 100, infoText));
    drawList.push_back(DrawablePointSize(44));
    image.draw(drawList);
    drawList.clear();

    char fname[13];
    sprintf(fname, "%08d.png", idx);
    image.magick("PNG");
    image.write(fname);
}

int main(int argc, char *argv[]){
    if(argc == 1){
        fprintf(stderr, "Not enough arguments.\n");
        display_help();
        return 1;
    }

    int c;
    opterr = 0;

    while((c = getopt(argc, argv, "hd:vl")) != -1){
        switch(c){
            case 'h':
                display_help();
                return 0;
            case 'd':
                if(isdigit(optarg[0])) hamming = atoi(optarg);
                else fprintf(stderr, "Invalid Hamming distance.\n");
                display_help();
                return 1;
                break;
            case 'v':
                verbose = true;
                break;
            case 'l':
                legacy = true;
                break;
            case '?':
                if(optopt == 'c') fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if(isprint(optopt)) fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                else fprintf(stderr, "Unknown option character `\\x%x'.\n",optopt);
                display_help();
                return 1;
            default:
                return 1;
        }
    }

    if(optind > argc+1){
        fprintf(stderr, "Too many arguments.\n");
        display_help();
        return 1;
    }

    InitializeMagick(*argv);
    if(legacy){
        if(verbose) printf("Generate original WhyCon marker\n");
        Image image = draw_whycon_markers();
        image.magick("PNG");
        image.write("00000000.png");
        printf("Rendering final image: =>  00000000.png\n");
        return 0;
    }

    if(!isdigit(argv[optind][0])){
        fprintf(stderr, "Invalid number of bits.\n");
        display_help();
        return 1;
    }

    const int teethCount = atoi(argv[optind]);
    CNecklace decoder = CNecklace(teethCount,hamming);
    int a[10000];
    n = decoder.printAll(a);
    if (decoder.verifyHamming(a,teethCount,n) < hamming){
        fprintf(stderr,"Hamming distance too low!\n");
        return 1;
    }
    w = 360.0/(float)teethCount/2.0;

    whycodeTorso = draw_whycon_markers();

    for(int i = 0; i < n; i++){
        draw_whycode_markers(a[i], i + 1, teethCount);
    }

    return 0;
}

