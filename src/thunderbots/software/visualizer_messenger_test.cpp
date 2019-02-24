#include "util/visualizer_messenger/visualizer_messenger.h"

using namespace Util;

int main(){

    for(;;){
        const auto vm = VisualizerMessenger::getInstance();

        auto drawAllShapes = [vm](double x, double y, VisualizerMessenger::VisualizerMessenger::DrawStyle style,
                                  VisualizerMessenger::DrawTransform transform) {
            //// Circle test
            //vm->drawEllipse("test", x + 40, y + 40, 40, 40, style, transform);

            //// Ellipse test
            //vm->drawEllipse("test", x + 200, y + 40, 40, 20, style, transform);

            //// Square test
            //vm->drawRect("test", x + 160, 20, 40, 40, style, transform);

            // Rectangle test
            vm->drawRect("test", x + 220, y + 20, 80, 20, style, transform);

            //// Arc test (0 to theta < pi)
            //vm->drawArc("test", x + 340, y + 40, 40, 0, 1.5, style, transform);

            //// Arc test (0 to theta > pi)
            //vm->drawArc("test", x + 400, y + 40, 40, 0, 4.5, style, transform);

            //// Arc test (0 to theta > 2 pi)
            //vm->drawArc("test", x + 460, y + 40, 40, 0, 7, style, transform);

            //// Arc test (0 to negative)
            //vm->drawArc("test", x + 520, y + 40, 40, 0, -0.5, style, transform);

            //// Arc test (negative to 0)
            //vm->drawArc("test", x + 580, y + 40, 40, -1.5, 0, style, transform);

            //// Arc test (negative to > pi)
            //vm->drawArc("test", x + 640, y + 40, 40, -4, 5, style, transform);

            //// Line test
            //vm->drawLine("test", x + 660, y + 20, 700, 40, style, transform);

        };


        //// Try different styles and transform
        VisualizerMessenger::DrawStyle standard_style = VisualizerMessenger::DrawStyle();

        //VisualizerMessenger::DrawStyle no_stroke_style     = VisualizerMessenger::DrawStyle();
        //no_stroke_style.stroke_weight = 0;

        //VisualizerMessenger::DrawStyle patriot_style     = VisualizerMessenger::DrawStyle();
        //patriot_style.fill          = "blue";
        //patriot_style.stroke        = "red";
        //patriot_style.stroke_weight = 5;

        VisualizerMessenger::DrawTransform standard_trans = VisualizerMessenger::DrawTransform();

        for (int i = 0; i < 100000; i++){
            drawAllShapes(50, 50, standard_style, standard_trans);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //VisualizerMessenger::DrawTransform enlarged_trans = VisualizerMessenger::DrawTransform();
        //enlarged_trans.scale         = 1.5;

        //VisualizerMessenger::DrawTransform shrunk_trans = VisualizerMessenger::DrawTransform();
        //shrunk_trans.scale         = 0.75;

        //VisualizerMessenger::DrawTransform rotated_trans = VisualizerMessenger::DrawTransform();
        //rotated_trans.rotation      = 0.52; /* ~60 deg */

        //// call the different things
        //drawAllShapes(0, 0, standard_style, standard_trans);
        //drawAllShapes(0, 100, standard_style, enlarged_trans);
        //drawAllShapes(0, 200, standard_style, shrunk_trans);
        //drawAllShapes(0, 300, standard_style, rotated_trans);

        //drawAllShapes(0, 400, no_stroke_style, standard_trans);
        //drawAllShapes(0, 500, no_stroke_style, enlarged_trans);
        //drawAllShapes(0, 600, no_stroke_style, shrunk_trans);
        //drawAllShapes(0, 700, no_stroke_style, rotated_trans);

        //drawAllShapes(0, 800, patriot_style, standard_trans);
        //drawAllShapes(0, 900, patriot_style, enlarged_trans);
        //drawAllShapes(0, 1000, patriot_style, shrunk_trans);
        //drawAllShapes(0, 1100, patriot_style, rotated_trans);

        vm->publishAndClearLayers();
    }
}