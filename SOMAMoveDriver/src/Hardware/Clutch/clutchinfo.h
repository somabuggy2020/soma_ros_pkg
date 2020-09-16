#ifndef CLUTCHINFO_H
#define CLUTCHINFO_H

#include <QString>
#include <QDebug>

/*!
 * namespace ClutchState
 */
namespace ClutchInfo
{
	const int Forward		= 1;
	const int Backward	= 2;
	const int Free			= 3;

	namespace Str{
		const QString Forward		= QString("Forward");
		const QString Backward	= QString("Backward");
		const QString Free			= QString("Free");
	}

	const QMap<int, QString> str{{Forward, "Forward"},
															 {Backward, "Backward"},
															 {Free, "Free"}};

	struct Data_t
	{
		int In;
		int Out;

		Data_t() : In(Free), Out(Free) {}
	};
}

#endif // CLUTCHINFO_H
