#include "my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "ros/ros.h"
#include "XR1.h"
#include <QTimer>
#include <QMessageBox>
#include <QLabel>
#include <QGroupBox>
#include <QFile>
#include <QTime>
#include <QTextStream>



#define PI 3.141592654
namespace vrep_test {




void MyPlugin::onBtnAddClicked()
{
  addAction(currentPosition, ui_.lineEdit->text().toDouble(), tr("newAction%1").arg(ui_.actionList->count()));
}

void MyPlugin::onBtnRemoveClicked()
{
  removeAction(ui_.actionList->currentRow());
}

std::vector<double> MyPlugin::processCurrents() {

  std::vector<double> res;

  for ( int i = 0; i < 10 ; i++) {

    vrep_test::JointCurrent srv;

    srv.request.NAME = "Left";

    if (CurrentClient.call(srv)) {
      ROS_INFO("Got Service Response from Vrep");
      res.push_back(srv.response.LSX);
      res.push_back(srv.response.LSY);
      res.push_back(srv.response.LEZ);
      res.push_back(srv.response.LEX);
      res.push_back(srv.response.LWZ);
      res.push_back(srv.response.LWY);
      res.push_back(srv.response.LWX);
      res.push_back(srv.response.COL);
    }

    delay(20);

  }
  return res;
}

void MyPlugin::onSave_CurrentClicked() {

  QString filename = "/home/rocky/CollectedData/testrun.txt";
  QFile fileout(filename);
  QTextStream out(&fileout);

  if (!fileout.open(QIODevice::ReadWrite)) ROS_INFO("Failed to write , plzz check privilege");
  else {
    for (int i = 0 ; i < GeneratedConfiguration.size() ; i++) {

      for ( int j = 0 ; j < GeneratedConfiguration[i].size() ; j++) out << GeneratedConfiguration[i][j] << " " ;
      for ( int j = 0 ; j < CurrentData[i].size() ; j++) out << CurrentData[i][j] << " " ;

      out << endl;
    }

  }

  fileout.close();
}


void MyPlugin::delay(int delay_time) {
  QTime dieTime = QTime::currentTime().addMSecs(delay_time);
  while (QTime::currentTime() < dieTime)
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}



void MyPlugin::onDance_ButtonClicked() {

  read_saved_path();
  ROS_INFO("Data Read");
  ROS_INFO("Size of the data is [%d]" , (int)m_cmdValue.size());
  if (!Path_Ex_Timer) {
    Path_Ex_Timer = new QTimer(this);

    connect(Path_Ex_Timer, SIGNAL(timeout()), this, SLOT(Path_Ex_Fun()));
  }
  else Path_Ex_Timer->stop();

  Path_idx = 0;
  Path_Ex_Timer->start(100);
}


void MyPlugin::read_saved_path() {

  while (m_cmdValue.size() > 0) m_cmdValue.pop_back();

  ROS_INFO("Reading Infos");
  QFile file("/home/rocky/CollectedData/path.txt");
  if (file.open(QFile::ReadOnly | QFile::Text))
  {
    while (!file.atEnd()) {
      QByteArray arr = file.readLine();
      arr.replace('\n', ' ');
      QList<QByteArray> arrList = arr.split(',');
      if (arrList.size() < 7)
      {
        ROS_INFO("Data Errors");
        continue;
      }
      std::vector<double> cmdValue;
      foreach (QByteArray tmp, arrList) {
        cmdValue.push_back(tmp.toDouble());
      }
      m_cmdValue.push_back(cmdValue);
    }
  }
}


void MyPlugin::Path_Ex_Fun() {

  ROS_INFO("Entered Step [%d]" , Path_idx);
  std::vector<double>  temp_angles = m_cmdValue[Path_idx];

  std::vector<double> targetPosition;

  for (int i = 0 ; i < 4 ; i++ )
    targetPosition.push_back(temp_angles[i]);

  while (targetPosition.size() < 21) targetPosition.push_back(0.0);

  targetPosition[7] = temp_angles[4];
  targetPosition[14] = temp_angles[6];

  JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(targetPosition));

  if (Path_idx < m_cmdValue.size() - 1) Path_idx++;
}

void MyPlugin::addAction(std::vector<double> &position, double time, QString actionName)
{
  QListWidgetItem * pItem = new QListWidgetItem(actionName, ui_.actionList);
  pItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
  m_Actions.push_back(position);
  time = time > 0 ? time : 5;
  m_ActionsTimes.push_back(time);
}

void MyPlugin::removeAction(int nActionIdx)
{
  if (nActionIdx < ui_.actionList->count())
  {
    QListWidgetItem * pItem = ui_.actionList->takeItem(nActionIdx);
    delete pItem;
    m_Actions.remove(nActionIdx);
    m_ActionsTimes.remove(nActionIdx);
  }
}

void MyPlugin::modifyAction()
{
  if (ui_.actionList->currentRow() +1)
  {
    m_Actions[ui_.actionList->currentRow()] = currentPosition;
    m_ActionsTimes[ui_.actionList->currentRow()] = ui_.lineEdit->text().toDouble();
  }
}

void MyPlugin::play()
{

  playing_switch = true;
  if (ui_.actionList->count() > 1 && ui_.actionList->currentRow() != ui_.actionList->count() - 1)
  {
    int currentidx;

    ROS_INFO("current idx is [%d]", currentidx);

    if (ui_.actionList->currentRow() + 1)
      currentidx = ui_.actionList->currentRow();
    else currentidx = 0;

    ROS_INFO("current idx is [%d]", currentidx);

    while ( currentidx < ui_.actionList->count()) {

      std::vector<double> goal_position = m_Actions.at(currentidx);
      double time = m_ActionsTimes.at(currentidx);

      ROS_INFO("We got this many joints [%d]", goal_position.size());

      ROS_INFO("We got this much time [%f]", time);

      std::vector<double> start_position;
      if (currentidx >  0)
        start_position = m_Actions.at(currentidx - 1);
      else {
        while (start_position.size() < goal_position.size())
          start_position.push_back(0.0);
      }

      ROS_INFO("We got this many joints [%d]", start_position.size());
      playcall_back(start_position, goal_position, time);
      currentidx++;
    }
  }
  playing_switch = false;
}

void MyPlugin::selectActionChanged(int nActionIdx)
{
  if (nActionIdx < ui_.actionList->count() && nActionIdx >= 0)
  {
    std::vector<double> position = m_Actions.at(nActionIdx);
    double time = m_ActionsTimes.at(nActionIdx);
    ui_.lineEdit->setText(tr("%1").arg(time));

    updateTargetSlider(position);
    //todo
  }
}

void MyPlugin::readAction()
{
  QFileDialog dialog(0, tr("Read Action"), QDir::currentPath());
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setNameFilter(tr("Action(*.action)"));
  if (dialog.exec() == QDialog::Accepted)
  {
    QString path = dialog.selectedFiles().first();
    if (path.size() > 0)
    {
      QFile file(path);
      if (file.open(QFile::ReadOnly | QFile::Text))
      {
        clearAction();
        QXmlStreamReader reader(&file);
        while (!reader.atEnd())
        {
          QXmlStreamReader::TokenType nType = reader.readNext();
          switch (nType) {
          case QXmlStreamReader::StartElement:
          {
            QString strName = reader.name().toString();
            if (strName == "Actions")
            {
              while (!reader.atEnd())
              {
                reader.readNextStartElement();
                QString actionName = reader.name().toString();
                reader.readNextStartElement();
                double time = reader.readElementText().toDouble();
                reader.readNextStartElement();
                int nCount = reader.readElementText().toUInt();
                if (nCount > 0)
                {
                  std::vector<double> position;
                  for (int i = 0; i < nCount; ++i)
                  {
                    reader.readNextStartElement();
                    position.push_back(reader.readElementText().toDouble());
                  }
                  addAction(position, time, actionName);
                }
                reader.readNextStartElement();
              }
            }
          }
          break;
          default:
            break;
          }
        }
      }
      file.close();
    }
  }
}

void MyPlugin::saveAction()
{
  if (ui_.actionList->count() < 2)
    return;
  QFileDialog dialog(0, tr("Save Action"), QDir::currentPath());
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  dialog.setNameFilter(tr("Action(*.action)"));
  if (dialog.exec() == QDialog::Accepted)
  {
    QString path = dialog.selectedFiles().first();
    if (path.size() > 0)
    {
      if (!path.endsWith(".action"))
      {
        path += ".action";
      }
      QFile file(path);
      if (file.open(QFile::WriteOnly | QFile::Text | QFile::Truncate))
      {
        QXmlStreamWriter writer(&file);
        writer.setAutoFormatting(true);
        writer.writeStartDocument("1.0", true);
        writer.writeStartElement("Actions");
        for (int i = 0; i < ui_.actionList->count(); ++i)
        {
          writer.writeStartElement(ui_.actionList->item(i)->text());
          writer.writeTextElement(tr("Time"), tr("%1").arg(m_ActionsTimes.at(i)));
          writer.writeTextElement(tr("Count"), tr("%1").arg(m_Actions.at(i).size()));
          for (int nPos = 0; nPos < m_Actions.at(i).size(); ++nPos)
          {
            // ROS_INFO("Writing row [%d] [%d]" , i , nPos);
            writer.writeTextElement(tr("Pose%1").arg(nPos), tr("%1").arg(m_Actions.at(i)[nPos]));
          }
          writer.writeEndElement();
        }
        writer.writeEndElement();
        writer.writeEndDocument();
      }
      file.close();
    }
  }
}

void MyPlugin::generateActuatorData()
{
  QFileDialog dialog(0, tr("Save Action"), QDir::currentPath());
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  dialog.setNameFilter(tr("ActuatorData(*.data)"));
  if (dialog.exec() == QDialog::Accepted)
  {
    QString path = dialog.selectedFiles().first();
    if (path.size() > 0)
    {
      QFile file(path);
      if (file.open(QFile::WriteOnly | QFile::Text | QFile::Truncate))
      {

      }
      file.close();
    }
  }
}

void MyPlugin::clearAction()
{
  m_Actions.clear();
  m_ActionsTimes.clear();
  int nRow = ui_.actionList->count();
  for (int i = nRow ; --i >= 0;)
  {
    QListWidgetItem * pItem = ui_.actionList->takeItem(0);
    delete pItem;
  }
}

void MyPlugin::updateTargetSlider(std::vector<double> v) {
  for (int i = 0; i < targetPositionSliders.size(); i++) {
    targetPositionSliders[i]->setValue((int) ((v[i] - joint_lower_limit[i]) / (joint_upper_limit[i] - joint_lower_limit[i]) * 100 ));
  }

  JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(v));
}

void MyPlugin::playcall_back( std::vector<double > start_position ,  std::vector<double> goal_position ,  double time) {


  int steps;
  steps = floor(time * 1000 / 10);

  ROS_INFO("We got this many steps [%d]" , steps);

  std::vector<double> increments;
  std::vector<double> intermediate_position = start_position;

  for (int i = 0 ; i < start_position.size() ; i++) {
    increments.push_back((goal_position[i] - start_position[i]) / (double)steps);
  }

  ROS_INFO("We got this many increments [%d]" , increments.size());

  for (int i = 0 ; i < steps + 1 ; i++) {
    for (int j = 0 ; j < goal_position.size() ; j++)
      intermediate_position[j] = increments[j] + intermediate_position[j];


    delay(10);

    updateTargetSlider(intermediate_position);
  }

}


} // namespace


PLUGINLIB_DECLARE_CLASS(vrep_test, MyPlugin, vrep_test::MyPlugin, rqt_gui_cpp::Plugin)
