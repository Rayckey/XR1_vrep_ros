#include "my_plugin.h"



#define PI 3.141592654
namespace vrep_test {




void MyPlugin::onBtnAddClicked()
{
  addAction(currentPosition, ui_.lineEdit->text().toDouble(), tr("newAction%1").arg(ui_.actionList->count()));
  addHandAction(getHandTargetPositions());
  addOmniAction(getOmniAction());
}

void MyPlugin::onBtnRemoveClicked()
{
  removeAction(ui_.actionList->currentRow());
}

// std::vector<double> MyPlugin::processCurrents() {

//   std::vector<double> res;

//   for ( int i = 0; i < 10 ; i++) {

//     vrep_test::JointCurrent srv;

//     srv.request.NAME = "Left";

//     if (CurrentClient.call(srv)) {
//       ROS_INFO("Got Service Response from Vrep");
//       res.push_back(srv.response.LSX);
//       res.push_back(srv.response.LSY);
//       res.push_back(srv.response.LEZ);
//       res.push_back(srv.response.LEX);
//       res.push_back(srv.response.LWZ);
//       res.push_back(srv.response.LWY);
//       res.push_back(srv.response.LWX);
//       res.push_back(srv.response.COL);
//     }

//     delay(20);

//   }
//   return res;
// }

// void MyPlugin::onSave_CurrentClicked() {

//   QString filename = "/home/rocky/CollectedData/testrun.txt";
//   QFile fileout(filename);
//   QTextStream out(&fileout);

//   if (!fileout.open(QIODevice::ReadWrite)) ROS_INFO("Failed to write , plzz check privilege");
//   else {
//     for (int i = 0 ; i < GeneratedConfiguration.size() ; i++) {

//       for ( int j = 0 ; j < GeneratedConfiguration[i].size() ; j++) out << GeneratedConfiguration[i][j] << " " ;
//       for ( int j = 0 ; j < CurrentData[i].size() ; j++) out << CurrentData[i][j] << " " ;


//       out << endl;
//     }

//   }

//   fileout.close();
// }


void MyPlugin::delay(int delay_time) {
  QTime dieTime = QTime::currentTime().addMSecs(delay_time);
  while (QTime::currentTime() < dieTime)
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}



void MyPlugin::onDance_ButtonClicked() {

  read_saved_path();
  ROS_INFO("Data Read");
  ROS_INFO("Size of the data is [%d]" , (int)GeneratedConfiguration.size());

  Path_Ex_Timer->stop();

  OmniPositions[0] = 0.0;
  OmniPositions[1] = 0.0;
  OmniPositions[2] = 0.0;

  Path_idx = 0;
  Path_Ex_Timer->start(10);

}


void MyPlugin::read_saved_path() {

  while (GeneratedConfiguration.size() > 0) GeneratedConfiguration.pop_back();

  ROS_INFO("Reading Infos");


  QFileDialog dialog(0, tr("Read Action"), QDir::currentPath());
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setNameFilter(tr("Data(*.data)"));
  if (dialog.exec() == QDialog::Accepted)
  {
    QString path = dialog.selectedFiles().first();
    if (path.size() > 0)
    {
      QFile file(path);
      if (file.open(QFile::ReadOnly | QFile::Text))
      {
        clearAction();
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
          GeneratedConfiguration.push_back(cmdValue);
        }
      }
    }
  }
}


void MyPlugin::Path_Ex_Fun() {

  playing_switch = true;

  ROS_INFO("Entered Step [%d]" , Path_idx);
  std::vector<double>  temp_angles = GeneratedConfiguration[Path_idx];

  std::vector<double> targetPosition;

  for (int i = 0 ; i < 21 ; i++ )
    targetPosition.push_back(temp_angles[i]);

  std::vector<double> handtargetPosition;

  for (int i = 21 ; i < 31 ; i++)
    handtargetPosition.push_back(temp_angles[i]);

  updateTargetSlider(targetPosition, handtargetPosition);

  // JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(targetPosition));

  OmniPositions[0] += temp_angles[31] * 0.01;
  OmniPositions[1] += temp_angles[32] * 0.01;
  OmniPositions[2] += temp_angles[33] * 0.01;

  //  geometry_msgs::Twist msg;

  // msg.linear.x = OmniPositions[0];
  // msg.linear.y = OmniPositions[1];
  // msg.angular.z = OmniPositions[2];

  // TwistPublisher.publish(msg);

  if (Path_idx < GeneratedConfiguration.size() - 1) Path_idx++;
  else {
    playing_switch = false;
    Path_Ex_Timer->stop();
  }
}

void MyPlugin::addAction(std::vector<double> &position, double time, QString actionName)
{
  QListWidgetItem * pItem = new QListWidgetItem(actionName, ui_.actionList);
  pItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
  m_Actions.push_back(position);
  time = time > 0 ? time : 5;
  m_ActionsTimes.push_back(time);
  // m_ActionsHands.push_back(hand_position);
}

void MyPlugin::addHandAction(std::vector<double> position)
{
  m_ActionsHands.push_back(position);
  // m_ActionsHands.push_back(hand_position);
}


void MyPlugin::removeAction(int nActionIdx)
{
  if (nActionIdx < ui_.actionList->count())
  {
    QListWidgetItem * pItem = ui_.actionList->takeItem(nActionIdx);
    delete pItem;
    m_Actions.remove(nActionIdx);
    m_ActionsTimes.remove(nActionIdx);
    m_ActionsHands.remove(nActionIdx);
    m_ActionsOmni.remove(nActionIdx);
  }
}

void MyPlugin::modifyAction()
{
  if (ui_.actionList->currentRow() + 1)
  {
    m_Actions[ui_.actionList->currentRow()] = currentPosition;
    m_ActionsTimes[ui_.actionList->currentRow()] = ui_.lineEdit->text().toDouble();
    m_ActionsHands[ui_.actionList->currentRow()] = getHandTargetPositions();
    m_ActionsOmni[ui_.actionList->currentRow()] = getOmniAction();
  }
}

void MyPlugin::play()
{

  ui_.tabWidget->setCurrentIndex(0);

  generateActuatorDataHelper();

  playing_switch = true;



  Path_Ex_Timer->stop();

  Path_idx = 0;

  OmniPositions[0] = 0.0;
  OmniPositions[1] = 0.0;
  OmniPositions[2] = 0.0;
  Path_Ex_Timer->start(10);




  // if (ui_.actionList->count() > 1 && ui_.actionList->currentRow() != ui_.actionList->count() - 1)
  // {
  //   int currentidx;

  //   // // ROS_INFO("current idx is [%d]", currentidx);

  //   // if (ui_.actionList->currentRow() + 1)
  //   //   currentidx = ui_.actionList->currentRow();
  //   // else currentidx = 0;

  //   // // ROS_INFO("current idx is [%d]", currentidx);

  //   // while ( currentidx < ui_.actionList->count()) {

  //   //   std::vector<double> goal_position = m_Actions.at(currentidx);
  //   //   std::vector<double> goal_hand_position = m_ActionsHands.at(currentidx);
  //   //   double time = m_ActionsTimes.at(currentidx);

  //   //   // ROS_INFO("We got this many joints [%d]", goal_position.size());

  //   //   // ROS_INFO("We got this much time [%f]", time);

  //   //   std::vector<double> start_position;
  //   //   std::vector<double> start_hand_position;

  //   //   if (currentidx >  0) {
  //   //     start_hand_position = m_ActionsHands.at(currentidx - 1);
  //   //     start_position = m_Actions.at(currentidx - 1);
  //   //   }
  //   //   else {
  //   //     while (start_position.size() < goal_position.size())
  //   //       start_position.push_back(0.0);
  //   //     while (start_hand_position.size() < goal_hand_position.size())
  //   //       start_hand_position.push_back(0.0);
  //   //   }

  //   //   // ROS_INFO("We got this many joints [%d]", start_position.size());
  //   //   playcall_back(start_position, goal_position, start_hand_position, goal_hand_position, time);
  //   //   currentidx++;
  //   // }
  // }
  // playing_switch = false;
}

void MyPlugin::selectActionChanged(int nActionIdx)
{
  if (nActionIdx < ui_.actionList->count() && nActionIdx >= 0)
  {
    std::vector<double> position = m_Actions.at(nActionIdx);
    std::vector<double> hand_position = m_ActionsHands.at(nActionIdx);
    double time = m_ActionsTimes.at(nActionIdx);
    ui_.lineEdit->setText(tr("%1").arg(time));


    std::vector<double> speedo = m_ActionsOmni.at(nActionIdx);
    ui_.lineEdit_X->setText(tr("%1").arg(speedo[0]));
    ui_.lineEdit_Y->setText(tr("%1").arg(speedo[1]));
    ui_.lineEdit_Z->setText(tr("%1").arg(speedo[2]));

    updateTargetSlider(position , hand_position);
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
                //Read em body
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

                // Read em hands
                reader.readNextStartElement();
                nCount = reader.readElementText().toUInt();
                if (nCount > 0)
                {
                  std::vector<double> position;
                  for (int i = 0; i < nCount; ++i)
                  {
                    reader.readNextStartElement();
                    position.push_back(reader.readElementText().toDouble());
                  }
                  addHandAction(position);
                }


                // Read em omniwheels
                reader.readNextStartElement();
                nCount = reader.readElementText().toUInt();
                if (nCount > 0)
                {
                  std::vector<double> velocity;
                  for (int i = 0; i < nCount; ++i)
                  {
                    reader.readNextStartElement();
                    velocity.push_back(reader.readElementText().toDouble());
                  }
                  addOmniAction(velocity);
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

          //Body Action
          writer.writeTextElement(tr("Count"), tr("%1").arg(m_Actions.at(i).size()));
          for (int nPos = 0; nPos < m_Actions.at(i).size(); ++nPos)
          {
            // ROS_INFO("Writing row [%d] [%d]" , i , nPos);
            writer.writeTextElement(tr("Pose%1").arg(nPos), tr("%1").arg(m_Actions.at(i)[nPos]));
          }

          //Hands Action
          writer.writeTextElement(tr("Count"), tr("%1").arg(m_ActionsHands.at(i).size()));
          for (int nPos = 0; nPos < m_ActionsHands.at(i).size(); ++nPos)
          {
            // ROS_INFO("Writing row [%d] [%d]" , i , nPos);
            writer.writeTextElement(tr("HandPose%1").arg(nPos), tr("%1").arg(m_ActionsHands.at(i)[nPos]));
          }


          writer.writeTextElement(tr("Count"), tr("%1").arg(m_ActionsOmni.at(i).size()));
          for (int nPos = 0; nPos < m_ActionsOmni.at(i).size(); ++nPos)
          {
            // ROS_INFO("Writing row [%d] [%d]" , i , nPos);
            writer.writeTextElement(tr("OmniVel%1").arg(nPos), tr("%1").arg(m_ActionsOmni.at(i)[nPos]));
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

  generateActuatorDataHelper() ;

  if (GeneratedConfiguration.size()) {
    QFileDialog dialog(0, tr("Save Action"), QDir::currentPath());
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setNameFilter(tr("ActuatorData(*.data)"));

    if (dialog.exec() == QDialog::Accepted)
    {
      QString path = dialog.selectedFiles().first();
      if (path.size() > 0)
      {

        if (!path.endsWith(".data"))
        {
          path += ".data";
        }
        QFile file(path);
        if (file.open(QFile::WriteOnly | QFile::Text | QFile::Truncate))
        {
          QTextStream out(&file);

          for (int i = 0 ; i < GeneratedConfiguration.size() ; i++) {
            for (int j = 0 ; j < GeneratedConfiguration[0].size() - 1; j++) {
              out << GeneratedConfiguration[i][j] << ",";
            }
            out << GeneratedConfiguration[i][GeneratedConfiguration[0].size() - 1];
            out <<  endl;
          }
        }
        file.close();
      }
    }
  }


}

void MyPlugin::clearAction()
{
  m_Actions.clear();
  m_ActionsTimes.clear();
  m_ActionsHands.clear();
  m_ActionsOmni.clear();
  int nRow = ui_.actionList->count();
  ROS_INFO("clear Count [%d]" , nRow);
  ui_.actionList->clear();
//  for (int i = nRow ; --i >= 0;)
//  {
//    QListWidgetItem * pItem = ui_.actionList->takeItem(0);
//    delete pItem;
//  }
}

void MyPlugin::updateTargetSlider(std::vector<double> v , std::vector<double> u) {
  for (int i = 0; i < targetPositionSliders.size(); i++) {
    targetPositionSliders[i]->setValue((int) ((v[i] - joint_lower_limit[i]) / (joint_upper_limit[i] - joint_lower_limit[i]) * 100 ));
  }

  double left_hand_temp[5]; double right_hand_temp[5];
  for (int i = 0; i < HandPositionSliders[0].size(); i++) {
    HandPositionSliders[0][i]->setValue((int) ((u[i] - hand_joint_lower_limit[i]) / (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) * 100 ));
    HandPositionSliders[1][i]->setValue((int) ((u[i + 5] - hand_joint_lower_limit[i]) / (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) * 100 ));

    left_hand_temp[i] = u[i];
    right_hand_temp[i] = u[i + 5];
  }

  JointTargetPositionPublisher.publish(ConvertJointAnglesMsgs(v));

  LeftHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(left_hand_temp));
  RightHandJointTargetPositionPublisher.publish(ConvertHandJointAngleMsgs(right_hand_temp));
}

// void MyPlugin::playcall_back( std::vector<double > start_position ,  std::vector<double> goal_position ,
//                               std::vector<double > start_hand_position ,  std::vector<double> goal_hand_position , double time) {

//   int steps;
//   steps = floor(time * 1000 / 10);

//   // ROS_INFO("We got this many steps [%d]" , steps);

//   std::vector<double> increments;
//   std::vector<double> intermediate_position = start_position;

//   std::vector<double> increments_hands;
//   std::vector<double> intermediate_hand_position = start_hand_position;


//   for (int i = 0 ; i < start_position.size() ; i++) {
//     increments.push_back((goal_position[i] - start_position[i]) / (double)steps);
//   }

//   for (int i = 0 ; i < start_hand_position.size() ; i++) {
//     increments_hands.push_back((goal_hand_position[i] - start_hand_position[i]) / (double)steps);
//   }

//   // ROS_INFO("We got this many increments [%d]" , increments.size());

//   for (int i = 0 ; i < steps + 1 ; i++) {
//     for (int j = 0 ; j < goal_position.size() ; j++)
//       intermediate_position[j] = increments[j] + intermediate_position[j];
//     for (int j = 0 ; j < goal_hand_position.size() ; j++)
//       intermediate_hand_position[j] = increments_hands[j] + intermediate_hand_position[j];

//     delay(10);

//     updateTargetSlider(intermediate_position , intermediate_hand_position);
//   }

// }



void MyPlugin::generateActuatorDataHelper() {

  while (GeneratedConfiguration.size() > 0) GeneratedConfiguration.pop_back();

  int currentidx = 0;

  while ( currentidx < ui_.actionList->count()) {


    //For all the major joints
    std::vector<double> goal_position = m_Actions.at(currentidx);
    double time = m_ActionsTimes.at(currentidx);

    std::vector<double> start_position;

    if (currentidx >  0)
      start_position = m_Actions.at(currentidx - 1);
    else {
      while (start_position.size() < goal_position.size())
        start_position.push_back(0.0);
    }

    int steps;

    steps = floor(time * 1000 / 10);

    std::vector<double> increments;

    for (int i = 0 ; i < start_position.size() ; i++) {
      increments.push_back((goal_position[i] - start_position[i]) / (double)steps);
    }



    //For all the hand joints
    std::vector<double> start_hand_position;
    std::vector<double> goal_hand_position = m_ActionsHands.at(currentidx);

    if (currentidx >  0) {
      start_hand_position = m_ActionsHands.at(currentidx - 1);
    }
    else {
      while (start_hand_position.size() < goal_hand_position.size())
        start_hand_position.push_back(0.0);
    }
    std::vector<double> increments_hands;


    for (int i = 0 ; i < start_hand_position.size() ; i++) {
      increments_hands.push_back((goal_hand_position[i] - start_hand_position[i]) / (double)steps);
    }



    //For the Omni wheels
    std::vector<double> start_vel;
    std::vector<double> goal_vel = m_ActionsOmni.at(currentidx);

    if (currentidx >  0) {
      start_vel = m_ActionsOmni.at(currentidx - 1);
    }
    else {
      while (start_vel.size() < goal_vel.size())
        start_vel.push_back(0.0);
    }
    std::vector<double> increments_vel;


    for (int i = 0 ; i < start_vel.size() ; i++) {
      increments_vel.push_back((goal_vel[i] - start_vel[i]) / (double)steps);
    }



    // saving data
    for (int i = 1 ; i < steps + 1 ; i++) {
      std::vector<double> temp_res;
      for (int j = 0 ; j < goal_position.size() ; j++)
        temp_res.push_back(increments[j] * (double)i + start_position[j]) ;

      for (int j = 0 ; j < goal_hand_position.size() ; j++)
        temp_res.push_back(increments_hands[j] * (double)i + start_hand_position[j]) ;

      for (int j = 0 ; j < goal_vel.size() ; j++)
        temp_res.push_back(increments_vel[j] * (double)i + start_vel[j]) ;

      GeneratedConfiguration.push_back(temp_res);
    }

    currentidx++;
  }


}


std::vector<double> MyPlugin::getHandTargetPositions() {

  std::vector<double> temp;
  for (int i = 0; i < 5; i++) {
    temp.push_back((double)HandPositionSliders[0][i]->value() / 100. * (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) + hand_joint_lower_limit[i]) ;
  }

  for (int i = 0; i < 5; i++) {
    temp.push_back((double)HandPositionSliders[1][i]->value() / 100. * (hand_joint_upper_limit[i] - hand_joint_lower_limit[i]) + hand_joint_lower_limit[i]) ;
  }

  return temp;
}


std::vector<double> MyPlugin::getOmniAction() {
  std::vector<double> res;
  res.push_back(ui_.lineEdit_X->text().toDouble());
  res.push_back(ui_.lineEdit_Y->text().toDouble());
  res.push_back(ui_.lineEdit_Z->text().toDouble());

  return res;
}


void MyPlugin::addOmniAction(std::vector<double> OmniAction) {
  m_ActionsOmni.push_back(OmniAction);
}


} // namespace


// PLUGINLIB_DECLARE_CLASS(vrep_test, MyPlugin, vrep_test::MyPlugin, rqt_gui_cpp::Plugin)
