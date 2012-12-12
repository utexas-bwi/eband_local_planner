#include <QApplication>
#include <QFont>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <segbot_controller/qnode.hpp>

class MyWidget : public QWidget
{
public:
    MyWidget(QWidget *parent = 0,int argc = 0, char *argv[]=0);
private:
    segbot_controller::QNode rosnode;
};
  MyWidget::MyWidget(QWidget *parent, int argc, char *argv[])
  : QWidget(parent),rosnode(argc,argv)
  {
    QPushButton *quit = new QPushButton(tr("Quit"));
    quit->setFont(QFont("Times", 18, QFont::Bold));

    QPushButton *stop = new QPushButton(tr("Stop"));
    stop->setFont(QFont("Times", 18, QFont::Bold));

    QPushButton *cont = new QPushButton(tr("Continue"));
    cont->setFont(QFont("Times", 18, QFont::Bold));

    connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));
    QObject::connect(stop, SIGNAL(clicked()), &rosnode, SLOT(stop()));
    QObject::connect(cont, SIGNAL(clicked()), &rosnode, SLOT(cont()));
                                                                                
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(stop);
    layout->addWidget(cont);
    layout->addWidget(quit);
    setLayout(layout);
  }

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  MyWidget widget(NULL, argc, argv);
  widget.show();
  return app.exec();
}
