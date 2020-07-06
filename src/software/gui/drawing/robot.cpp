#include "software/gui/drawing/robot.h"

#include <QtWidgets/QGraphicsEllipseItem>

#include "shared/constants.h"
#include "software/gui/drawing/geom.h"
#include "software/gui/geometry_conversion.h"
#include "software/new_geom/segment.h"
#include "software/math/math_functions.h"

void drawRobotVelocity(QGraphicsScene* scene, const Point& position,
                       const Vector& velocity, const QColor& slow_colour, const QColor& fast_colour)
{
    double speed = velocity.length();

    auto r = normalizeValueToRange<double>(speed, 0, ROBOT_MAX_SPEED_METERS_PER_SECOND, slow_colour.redF(), fast_colour.redF());
    auto g = normalizeValueToRange<double>(speed, 0, ROBOT_MAX_SPEED_METERS_PER_SECOND, slow_colour.greenF(), fast_colour.greenF());
    auto b = normalizeValueToRange<double>(speed, 0, ROBOT_MAX_SPEED_METERS_PER_SECOND, slow_colour.blueF(), fast_colour.blueF());
    auto colour = QColor::fromRgbF(r, g, b);

    QPen pen(colour);
    pen.setWidth(4);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    // A somewhat arbitrary value that we've determined looks nice in the GUI
    const double max_velocity_line_length = 0.5;
    auto line_length = normalizeValueToRange<double>(speed, 0, ROBOT_MAX_SPEED_METERS_PER_SECOND, 0.0, max_velocity_line_length);

    drawSegment(scene, Segment(position, position + velocity.normalize(line_length)), pen);
}

void drawRobotAtPosition(QGraphicsScene* scene, const Point& position,
                         const Angle& orientation, const QColor& color)
{
    QPen pen(Qt::black);
    pen.setWidth(1);
    pen.setCosmetic(true);

    QBrush brush(color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    // This defines the rectangle that will "clip" or cover part of the robot ellipse.
    // This is what allows us to easily draw the flat front of the robot. We create an
    // invisible smaller "window" rectangle that the robot ellipse is shown through
    Point robot_clipping_bounding_box_top_left =
        position + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_clipping_bounding_box_bottom_right =
        position + Vector(DIST_TO_FRONT_OF_ROBOT_METERS, -ROBOT_MAX_RADIUS_METERS);
    QRectF robot_clipping_bounding_box(
        createQPointF(robot_clipping_bounding_box_top_left),
        createQPointF(robot_clipping_bounding_box_bottom_right));
    QGraphicsRectItem* robot_clipping_rect =
        new QGraphicsRectItem(robot_clipping_bounding_box);
    robot_clipping_rect->setTransformOriginPoint(createQPointF(position));
    robot_clipping_rect->setRotation(orientation.toDegrees());
    robot_clipping_rect->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
    robot_clipping_rect->setPen(Qt::NoPen);
    robot_clipping_rect->setBrush(Qt::NoBrush);

    Point robot_bounding_box_top_left =
        position + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_bounding_box_bottom_right =
        position + Vector(ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS);
    // This robot ellipse graphics item is a child of the robot_clipping_rect above so
    // that is can be covered / clipped by the clipping rect
    QGraphicsEllipseItem* robot_ellipse =
        new QGraphicsEllipseItem(QRectF(createQPointF(robot_bounding_box_top_left),
                                        createQPointF(robot_bounding_box_bottom_right)),
                                 robot_clipping_rect);
    robot_ellipse->setPen(pen);
    robot_ellipse->setBrush(brush);

    scene->addItem(robot_clipping_rect);
}

void drawRobotId(QGraphicsScene* scene, const Point& position, const RobotId id)
{
    Point robot_bounding_box_top_left =
        position + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_bounding_box_bottom_right =
        position + Vector(ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS);
    QRectF robot_bounding_box(createQPointF(robot_bounding_box_top_left),
                              createQPointF(robot_bounding_box_bottom_right));

    QGraphicsSimpleTextItem* robot_id = new QGraphicsSimpleTextItem(QString::number(id));
    QFont sansFont("Helvetica [Cronyx]");
    sansFont.setPointSizeF(1);
    robot_id->setFont(sansFont);
    robot_id->setBrush(QBrush(Qt::black));

    // Scale the text down so it fits right below the robot being drawn. We use the width
    // to calculate the scaling so that we can always ensure the text will fit within the
    // width of the robot's bounding box, and won't overflow if the text gets too long. We
    // care less about the height and just allow it to scale along with the width.
    double scaling_factor =
        1.0 / (robot_id->boundingRect().width() / robot_bounding_box.width());
    // Flip the y-axis so the text show right-side-up. When we set up the GraphicsView
    // that contains the scene we apply a transformation to the y-axis so that Qt's
    // coordinate system matches ours and we can draw things without changing our
    // convention. Unfortunately this flips all text by default, so we need to flip it
    // back here.
    QTransform scale_and_invert_y_transform(scaling_factor, 0, 0, -scaling_factor, 0, 0);
    robot_id->setTransform(scale_and_invert_y_transform);

    // Place the text right under the robot
    robot_id->setPos(robot_bounding_box_top_left.x(),
                     robot_bounding_box_bottom_right.y());
    scene->addItem(robot_id);
}

void drawRobot(QGraphicsScene* scene, const RobotStateWithId& robot, const QColor& color)
{
    drawRobotAtPosition(scene, robot.robot_state.position(),
                        robot.robot_state.orientation(), color);
    drawRobotVelocity(scene, robot.robot_state.position(), robot.robot_state.velocity(),
                      robot_speed_slow_color, robot_speed_fast_color);
    drawRobotId(scene, robot.robot_state.position(), robot.id);

    // TODO: Show robot charge state
    // https://github.com/UBC-Thunderbots/Software/issues/1492
}

void drawRobot(QGraphicsScene* scene, const RobotDetection& robot, const QColor& color)
{
    drawRobotAtPosition(scene, robot.position, robot.orientation, color);
    drawRobotId(scene, robot.position, robot.id);
}
