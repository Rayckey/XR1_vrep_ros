/****************************************************************************
** Meta object code from reading C++ file 'actuatordata.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../actuatorController/actuatordata.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'actuatordata.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ActuatorData_t {
    QByteArrayData data[14];
    char stringdata0[179];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ActuatorData_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ActuatorData_t qt_meta_stringdata_ActuatorData = {
    {
QT_MOC_LITERAL(0, 0, 12), // "ActuatorData"
QT_MOC_LITERAL(1, 13, 18), // "acturalVauleChange"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 22), // "Actuator::ActuatorMode"
QT_MOC_LITERAL(4, 56, 2), // "Id"
QT_MOC_LITERAL(5, 59, 14), // "QVector<qreal>"
QT_MOC_LITERAL(6, 74, 6), // "values"
QT_MOC_LITERAL(7, 81, 18), // "currentErrorChange"
QT_MOC_LITERAL(8, 100, 8), // "nErrorId"
QT_MOC_LITERAL(9, 109, 22), // "activeModeSuccessfully"
QT_MOC_LITERAL(10, 132, 8), // "saveData"
QT_MOC_LITERAL(11, 141, 8), // "loadData"
QT_MOC_LITERAL(12, 150, 9), // "reconnect"
QT_MOC_LITERAL(13, 160, 18) // "requestActualValue"

    },
    "ActuatorData\0acturalVauleChange\0\0"
    "Actuator::ActuatorMode\0Id\0QVector<qreal>\0"
    "values\0currentErrorChange\0nErrorId\0"
    "activeModeSuccessfully\0saveData\0"
    "loadData\0reconnect\0requestActualValue"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ActuatorData[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   49,    2, 0x06 /* Public */,
       7,    1,   54,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    0,   57,    2, 0x0a /* Public */,
      10,    0,   58,    2, 0x0a /* Public */,
      11,    0,   59,    2, 0x0a /* Public */,
      12,    0,   60,    2, 0x0a /* Public */,
      13,    0,   61,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, QMetaType::Int,    8,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ActuatorData::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ActuatorData *_t = static_cast<ActuatorData *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->acturalVauleChange((*reinterpret_cast< Actuator::ActuatorMode(*)>(_a[1])),(*reinterpret_cast< QVector<qreal>(*)>(_a[2]))); break;
        case 1: _t->currentErrorChange((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 2: _t->activeModeSuccessfully(); break;
        case 3: _t->saveData(); break;
        case 4: _t->loadData(); break;
        case 5: _t->reconnect(); break;
        case 6: _t->requestActualValue(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QVector<qreal> >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ActuatorData::*_t)(Actuator::ActuatorMode , QVector<qreal> );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ActuatorData::acturalVauleChange)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (ActuatorData::*_t)(const int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ActuatorData::currentErrorChange)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject ActuatorData::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_ActuatorData.data,
      qt_meta_data_ActuatorData,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ActuatorData::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ActuatorData::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ActuatorData.stringdata0))
        return static_cast<void*>(const_cast< ActuatorData*>(this));
    return QObject::qt_metacast(_clname);
}

int ActuatorData::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void ActuatorData::acturalVauleChange(Actuator::ActuatorMode _t1, QVector<qreal> _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ActuatorData::currentErrorChange(const int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
struct qt_meta_stringdata_MotorDataMgr_t {
    QByteArrayData data[13];
    char stringdata0[142];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MotorDataMgr_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MotorDataMgr_t qt_meta_stringdata_MotorDataMgr = {
    {
QT_MOC_LITERAL(0, 0, 12), // "MotorDataMgr"
QT_MOC_LITERAL(1, 13, 11), // "dataChanged"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 9), // "nDeviceId"
QT_MOC_LITERAL(4, 36, 27), // "Actuator::ActuatorAttribute"
QT_MOC_LITERAL(5, 64, 6), // "attrId"
QT_MOC_LITERAL(6, 71, 5), // "value"
QT_MOC_LITERAL(7, 77, 12), // "errorOccured"
QT_MOC_LITERAL(8, 90, 6), // "erroId"
QT_MOC_LITERAL(9, 97, 9), // "errorInfo"
QT_MOC_LITERAL(10, 107, 16), // "setProxyCallback"
QT_MOC_LITERAL(11, 124, 8), // "nProxyId"
QT_MOC_LITERAL(12, 133, 8) // "bSuccess"

    },
    "MotorDataMgr\0dataChanged\0\0nDeviceId\0"
    "Actuator::ActuatorAttribute\0attrId\0"
    "value\0errorOccured\0erroId\0errorInfo\0"
    "setProxyCallback\0nProxyId\0bSuccess"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MotorDataMgr[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   29,    2, 0x06 /* Public */,
       7,    3,   36,    2, 0x06 /* Public */,
      10,    2,   43,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::UChar, 0x80000000 | 4, QMetaType::QVariant,    3,    5,    6,
    QMetaType::Void, QMetaType::UChar, QMetaType::UShort, QMetaType::QString,    3,    8,    9,
    QMetaType::Void, QMetaType::UChar, QMetaType::Bool,   11,   12,

       0        // eod
};

void MotorDataMgr::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MotorDataMgr *_t = static_cast<MotorDataMgr *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->dataChanged((*reinterpret_cast< const quint8(*)>(_a[1])),(*reinterpret_cast< const Actuator::ActuatorAttribute(*)>(_a[2])),(*reinterpret_cast< QVariant(*)>(_a[3]))); break;
        case 1: _t->errorOccured((*reinterpret_cast< const quint8(*)>(_a[1])),(*reinterpret_cast< const quint16(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 2: _t->setProxyCallback((*reinterpret_cast< const quint8(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MotorDataMgr::*_t)(const quint8 , const Actuator::ActuatorAttribute , QVariant );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MotorDataMgr::dataChanged)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (MotorDataMgr::*_t)(const quint8 , const quint16 , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MotorDataMgr::errorOccured)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (MotorDataMgr::*_t)(const quint8 , bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MotorDataMgr::setProxyCallback)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject MotorDataMgr::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_MotorDataMgr.data,
      qt_meta_data_MotorDataMgr,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MotorDataMgr::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MotorDataMgr::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MotorDataMgr.stringdata0))
        return static_cast<void*>(const_cast< MotorDataMgr*>(this));
    return QObject::qt_metacast(_clname);
}

int MotorDataMgr::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void MotorDataMgr::dataChanged(const quint8 _t1, const Actuator::ActuatorAttribute _t2, QVariant _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MotorDataMgr::errorOccured(const quint8 _t1, const quint16 _t2, QString _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MotorDataMgr::setProxyCallback(const quint8 _t1, bool _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
