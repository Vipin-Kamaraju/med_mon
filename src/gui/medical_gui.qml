import QtQuick 2.12
import QtQuick.Controls 2.12
import QtCharts 2
import QtQuick.Layouts 1.12

ApplicationWindow {
    visible: true
    width: 1600
    height: 1200
    title: "Medical Device Dashboard"

    property int x: 0
    property int x_width: 200

    GridLayout {
        anchors.fill: parent
        columns: 3

        // Column 1: Heart Rate and Blood Pressure
        ColumnLayout {
            Layout.column: 0
            Layout.row: 0

            Label {
                id: heartRateLabel
                text: "<b>Heart Rate:</b> -- bpm"
                font.pointSize: 20
            }

            Label {
                id: bloodPressureLabel
                text: "<b>Blood Pressure:</b> -- mmHg"
                font.pointSize: 20
            }
        }

        // Column 2: Checkbox
        ColumnLayout {
            Layout.column: 1
            Layout.row: 0
            CheckBox {
                id: filterEnabled
                text: "<b>Enable EKG Filter</b>"
                font.pointSize: 20 // Increased font size
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
        }

        // Column 3: Slider and Buttons
        ColumnLayout {
            Layout.column: 2
            Layout.row: 0

            Label {
                id: cutoffLabel
                text: "<b>EKG Filter Cutoff Frequency:</b> " + cutoffSlider.value.toFixed(2)
                font.pointSize: 20
            }

            RowLayout {
                Button {
                    text: "-"
                    font.pointSize: 25 // Increased font size
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
                        cutoffLabel.text = "<b>EKG Filter Cutoff Frequency:</b> " + value.toFixed(2);
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
        }

        // ChartView (spans all columns)
        ChartView {
            id: chartView
            Layout.row: 1
            Layout.columnSpan: 3
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.alignment: Qt.AlignTop
            antialiasing: true

            ValueAxis {
                id: axisX
                min: 0
                max: x_width
            }

            ValueAxis {
                id: axisY
                min: 690
                max: 930
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
            //console.log("[QML] Received Heart Rate:", heartRate)
            heartRateLabel.text = "<b>Heart Rate:</b> <font color='red'>" + heartRate + "</font> bpm"
        }

        onBloodPressureChanged: function(bloodPressure) {
            //console.log("[QML] Blood Pressure Changed:", bloodPressure)
            bloodPressureLabel.text = "<b>Blood Pressure:</b> <font color='red'>" + bloodPressure + "</font> mmHg"
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
         chartView.legend.font.pointSize = 16;
    }
}
