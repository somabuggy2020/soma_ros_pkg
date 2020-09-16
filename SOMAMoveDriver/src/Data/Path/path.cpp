#include "path.h"

Path::Path(QObject *parent) : QObject(parent)
{
	segments.clear();
}

void Path::set(QList<QLineF> path)
{
	Q_FOREACH(QLineF l, path){
		this->segments.push_back(l);
	}
}

void Path::append(QLineF segment)
{
	segments.push_back(segment);
}

QPointF Path::searchClosestPoitntOnPath(QPointF query, QLineF *_seg)
{
	QVector2D shortest = QVector2D(INFINITY, INFINITY);
	QLineF tmpSeg;

	Q_FOREACH(QLineF seg, segments){
		QVector2D a = QVector2D(seg.p2() - seg.p1());
		QVector2D b = QVector2D(query - seg.p1());

		double r = QVector2D::dotProduct(a, b)/a.lengthSquared();

		QPointF tmp;
		QVector2D tmp_v;

		if(r <= 0.0) tmp = seg.p1();
		else if(r >= 1.0) tmp = seg.p2();
		else{
			QVector2D p = QVector2D(seg.p1())	+ r*a;
			tmp = QPointF(p.x(), p.y());
		}

		tmp_v = QVector2D(tmp - query);

		if(tmp_v.length() < shortest.length()){
			shortest = tmp_v;
			tmpSeg = seg;
		}
	}

	(*_seg) = tmpSeg;
	return QPointF(query.x() + shortest.x(), query.y() + shortest.y());
}




