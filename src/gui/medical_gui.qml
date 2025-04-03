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

        CheckBox {
            id: filterEnabled
            text: "Enable EKG Filter"
            checked: false
            onCheckedChanged: {
                cutoffSlider.enabled = checked;
                if (checked) {
                    gui.update_cutoff(cutoffSlider.value);
                } else {
                    gui.update_cutoff(0.0); // Set cutoff to 0 to disable filter
                }
            }
        }

        Label {
            id: cutoffLabel
            text: "EKG Filter Cutoff Frequency: " + cutoffSlider.value.toFixed(2)
            font.pointSize: 12
        }

        Row {
            Button {
                text: "-"
                font.pointSize: 20 // Increased font size
                onClicked: {
                    cutoffSlider.value = Math.max(cutoffSlider.from, cutoffSlider.value - cutoffSlider.stepSize);
                    gui.update_cutoff(cutoffSlider.value);
                }
            }

            Slider {
                id: cutoffSlider
                from: 0.0  // Minimum cutoff frequency (filter disabled)
                to: 0.3     // Maximum cutoff frequency (filter enabled)
                value: 0.1   // Default cutoff frequency (filter disabled)
                stepSize: 0.01
                enabled: filterEnabled.checked
                onValueChanged: {
                    gui.update_cutoff(value);
                    cutoffLabel.text = "EKG Filter Cutoff Frequency: " + value.toFixed(2);
                }
            }

            Button {
                text: "+"
                font.pointSize: 20 // Increased font size
                onClicked: {
                    cutoffSlider.value = Math.min(cutoffSlider.to, cutoffSlider.value + cutoffSlider.stepSize);
                    gui.update_cutoff(cutoffSlider.value);
                }
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
            // console.log("[QML] Received Heart Rate:", heartRate)
            heartRateLabel.text = "Heart Rate: " + heartRate + " bpm"
        }

        onBloodPressureChanged: function(bloodPressure) {
            // console.log("[QML] Blood Pressure Changed:", bloodPressure)
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
