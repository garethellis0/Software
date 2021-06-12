#include "software/gui/drawing/obstacle_artist.h"

// TODO: move along with circleslice visit impl
#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsSimpleTextItem>
#include <QtGui/QPainter>
#include "software/gui/drawing/colors.h"
#include "software/geom/algorithms/acute_angle.h"

// TODO: move this else where
class QGraphicsArcItem : public QGraphicsEllipseItem {
public:
    QGraphicsArcItem ( QRectF rect, QGraphicsItem * parent = 0 ) :
            QGraphicsEllipseItem(rect, parent) {
    }

protected:
    void paint ( QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget) {
        painter->setPen(pen());
        painter->setBrush(brush());
        painter->drawArc(rect(), startAngle(), spanAngle());
    }
};

ObstacleArtist::ObstacleArtist(QGraphicsScene* scene, const QPen& pen)
    : scene_(scene), pen_(pen)
{
}

void ObstacleArtist::visit(const GeomObstacle<Circle>& geom_obstacle)
{
    drawCircle(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visit(const GeomObstacle<Polygon>& geom_obstacle)
{
    drawPolygon(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visit(const CircleWithSliceRemoved& circle_with_slice_removed) {
    // TODO: this should be it's own function in geom.h/geom.cpp
    // TODO: implement
    const Vector bounding_box_corner_offset(circle_with_slice_removed.radius(), circle_with_slice_removed.radius());
    Point bounding_box_top_left = circle_with_slice_removed.origin() + bounding_box_corner_offset;
    Point bounding_box_bottom_right = circle_with_slice_removed.origin() -
            bounding_box_corner_offset;

    auto [p1,p2] = circle_with_slice_removed.sliceCircleIntersectionPoints();

    // TODO: better naming, this is copy-pasted
    pen_.setCosmetic(true);
    QGraphicsEllipseItem* body_ellipse =
            new QGraphicsArcItem(QRectF(createQPointF(bounding_box_top_left),
                                            createQPointF(bounding_box_bottom_right)));
    body_ellipse->setPen(pen_);
    body_ellipse->setStartAngle(createQAngle((p2 - circle_with_slice_removed.origin()).orientation()));
    Angle body_ellipse_span =
            Angle::full() - acuteAngle(p2 - circle_with_slice_removed.origin(), p1 - circle_with_slice_removed.origin());
    body_ellipse->setSpanAngle(createQAngle(body_ellipse_span));
    scene_->addItem(body_ellipse);

    const Segment s1(p1, circle_with_slice_removed.sliceTip());
    const Segment s2(p2, circle_with_slice_removed.sliceTip());
    drawSegment(scene_, s1, pen_);
    drawSegment(scene_, s2, pen_);
}
