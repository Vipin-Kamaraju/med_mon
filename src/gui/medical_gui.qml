import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15

ApplicationWindow {
    visible: true
    width: 800
    height: 600
    title: "Medical Device Dashboard"

    property int x: 0
    property int x_width: 200

    Column {
        anchors.centerIn: parent

        CheckBox {
            id: filterToggle
            text: "Enable EKG Filter"
            checked: true
            onCheckedChanged: {
                gui.toggle_filter(checked)
            }
        }

        Label {
            id: heartRateLabel
            text: "Heart Rate: -- bpm"
            font.pointSize: 20
        }

        Label {
            id: bloodPressureLabel
            text: "Blood Pressure: -- mmHg"
            font.pointSize: 20
        }

        ChartView {
            id: chartView
            width: parent.width
            height: 300
            antialiasing: true

            ValueAxis {
                id: axisX
                min: 0
                max: x_width
            }

            ValueAxis {
                id: axisY
                min: 725
                max: 820
            }

            LineSeries {
                id: ekgSeries
                name: "EKG"
                useOpenGL: false
                axisX: axisX
                axisY: axisY
            }
        }
    }

    Connections {
        target: gui
        function onHeartRateChanged(heartRate) {
            heartRateLabel.text = "Heart Rate: " + heartRate + " bpm";
        }
        function onBloodPressureChanged(bloodPressure) {
            bloodPressureLabel.text = "Blood Pressure: " + bloodPressure + " mmHg";
        }
        function onEkgChanged(voltage) {
            ekgSeries.append(x, voltage);
            x = (x + 1) % x_width;

            if (ekgSeries.count > x_width) {
                ekgSeries.remove(0);
            }
        }
    }
}
