import QtQuick 2.12
import QtQuick.Controls 2.12
import QtCharts 2

ApplicationWindow {
    visible: true
    width: 1600
    height: 1200
    title: "Medical Device Dashboard"

    property int x: 0
    property int x_width: 200

    Column {
        anchors.centerIn: parent

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
            height: 600
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
        
        onHeartRateChanged: function(heartRate) {
            console.log("[QML] Received Heart Rate:", heartRate)
            heartRateLabel.text = "Heart Rate: " + heartRate + " bpm"
        }

        onBloodPressureChanged: function(bloodPressure) {
            console.log("[QML] Blood Pressure Changed:", bloodPressure)
            bloodPressureLabel.text = "Blood Pressure: " + bloodPressure + " mmHg"
        }

        onEkgChanged: function(voltage) {
            ekgSeries.append(x, voltage);
            x = (x + 1) % x_width;

            if (ekgSeries.count > x_width) {
                ekgSeries.remove(0);
            }
        }
    }

    Component.onCompleted: {
        console.log("gui object in QML is", gui)
    }
}
