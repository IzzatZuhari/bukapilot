#include "selfdrive/ui/qt/spinner.h"

#include <cstdio>
#include <iostream>
#include <string>

#include <QApplication>
#include <QGridLayout>
#include <QPainter>
#include <QString>
#include <QTransform>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"

TrackWidget::TrackWidget(QWidget *parent) : QWidget(parent) {
  setFixedSize(spinner_size);
  setAutoFillBackground(true);
  setPalette(Qt::black);

  // pre-compute all the track imgs. make this a gif instead?
  QTransform transform(1, 0, 0, 1, width() / 2, height() / 2);
  int img_count=0;
  for (QPixmap &img : track_imgs) {
    QString img_dir = "../assets/spinner/img_kommu_";
    std::string s_img_count = std::to_string(img_count);
    img_dir.append(s_img_count.c_str());
    img_dir.append(".jpg");
    img = QPixmap(img_dir).scaled(spinner_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    img_count++;
  }

  m_anim.setDuration(1000);
  m_anim.setStartValue(0);
  m_anim.setEndValue(int(track_imgs.size() -1));
  m_anim.setLoopCount(-1);
  m_anim.start();
  connect(&m_anim, SIGNAL(valueChanged(QVariant)), SLOT(update()));
}

void TrackWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.drawPixmap(0, 0, track_imgs[m_anim.currentValue().toInt()]);
}

// Spinner

Spinner::Spinner(QWidget *parent) : QWidget(parent) {
  QGridLayout *main_layout = new QGridLayout(this);
  main_layout->setSpacing(0);
  main_layout->setMargin(200);

  main_layout->addWidget(new TrackWidget(this), 0, 0, Qt::AlignHCenter | Qt::AlignVCenter);

  text = new QLabel();
  text->setVisible(false);
  main_layout->addWidget(text, 1, 0, Qt::AlignHCenter);

  progress_bar = new QProgressBar();
  progress_bar->setRange(5, 100);
  progress_bar->setTextVisible(false);
  progress_bar->setVisible(false);
  progress_bar->setFixedHeight(20);
  main_layout->addWidget(progress_bar, 1, 0, Qt::AlignHCenter);

  setStyleSheet(R"(
    Spinner {
      background-color: black;
    }
    QLabel {
      color: white;
      font-size: 80px;
      background-color: transparent;
    }
    QProgressBar {
      background-color: #373737;
      width: 1000px;
      border solid white;
      border-radius: 10px;
    }
    QProgressBar::chunk {
      border-radius: 10px;
      background-color: white;
    }
  )");

  notifier = new QSocketNotifier(fileno(stdin), QSocketNotifier::Read);
  QObject::connect(notifier, &QSocketNotifier::activated, this, &Spinner::update);
};

void Spinner::update(int n) {
  std::string line;
  std::getline(std::cin, line);

  if (line.length()) {
    bool number = std::all_of(line.begin(), line.end(), ::isdigit);
    text->setVisible(!number);
    progress_bar->setVisible(number);
    text->setText(QString::fromStdString(line));
    if (number) {
      progress_bar->setValue(std::stoi(line));
    }
  }
}

int main(int argc, char *argv[]) {
  initApp();
  QApplication a(argc, argv);
  Spinner spinner;
  setMainWindow(&spinner);
  return a.exec();
}
