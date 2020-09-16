#ifndef PATH_H
#define PATH_H

#include <QObject>
#include <QLineF>
#include <QPointF>
#include <QLineF>
#include <QVector2D>

class Path : public QObject
{
	Q_OBJECT
public:
	explicit Path(QObject *parent = nullptr);

	void set(QList<QLineF> segments);
	void append(QLineF segment);

	QPointF searchClosestPoitntOnPath(QPointF query, QLineF *seg);	//query point

signals:

public slots:

public:
	QList<QLineF> segments;
	int cur_seg_idx;
};

#endif // PATH_H
