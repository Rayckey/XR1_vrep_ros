#ifndef SERIALAUTORECOGNIZE_H
#define SERIALAUTORECOGNIZE_H
#include "AbstractAutoRecognize.h"


class SerialAutoRecognize : public AbstractAutoRecognize
{
    Q_OBJECT
public:
    explicit SerialAutoRecognize(QObject *parent = 0);
    void startRecognize()override;
    void waitTimeout()override;
private:
    void findAvailablePorts();
private:
    QList <int> m_portList;
};

#endif // SERIALAUTORECOGNIZE_H
