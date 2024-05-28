import 'package:flutter/material.dart';
import 'package:flutter_blue/flutter_blue.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Bluetooth Example',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: const MyHomePage(),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  FlutterBlue flutterBlue = FlutterBlue.instance;
  BluetoothDevice? device;
  BluetoothCharacteristic? characteristic;

  @override
  void initState() {
    super.initState();
    connectToDevice();
  }

  void connectToDevice() async {
    // Start scanning
    flutterBlue.startScan(timeout: const Duration(seconds: 4));

    // Listen to scan results
    var subscription = flutterBlue.scanResults.listen((results) async {
      for (ScanResult r in results) {
        if (r.device.name == "BLUETOOTH HC-06") {  // Replace with your Bluetooth device name
          // Stop scanning
          flutterBlue.stopScan();

          // Connect to the device
          device = r.device;
          await device?.connect();

          // Discover services
          List<BluetoothService> services = await device!.discoverServices();
          for (BluetoothService service in services) {
            for (BluetoothCharacteristic c in service.characteristics) {
              // Assuming the characteristic you want is the first one
              characteristic = c;
              break;
            }
          }
          setState(() {});
          break;
        }
      }
    });
  }

  void sendData(String data) async {
    if (characteristic != null) {
      await characteristic?.write(data.codeUnits);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Flutter Bluetooth Example"),
      ),
      body: Center(
        child: GestureDetector(
          onTapDown: (_) {
            sendData('1');  // Button pressed
          },
          child: Container(
            width: 200,
            height: 100,
            color: Colors.blue,
            child: const Center(
              child: Text(
                "Press Me",
                style: TextStyle(color: Colors.white, fontSize: 24),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

