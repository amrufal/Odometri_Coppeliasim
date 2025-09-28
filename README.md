# Simulasi Odometri Wheeled Mobile Robot Pioneer P3DX

## 📌 Deskripsi
Proyek ini adalah simulasi **odometri pada robot mobile differential-drive (Pioneer P3DX)** menggunakan **CoppeliaSim Edu** dan **Python** dengan perantara **ZMQ Remote API**.  
Program ini menghitung posisi dan orientasi robot berdasarkan data kecepatan roda, membandingkan hasil odometri dengan data **ground truth** dari simulasi, dan menampilkan hasilnya dalam bentuk grafik serta metrik error kuantitatif :contentReference[oaicite:1]{index=1}.

## 🎯 Tujuan
- Mempelajari cara kerja metode odometri dalam memperkirakan pose robot.
- Mengimplementasikan metode odometri dengan bahasa Python.
- Merancang program yang menghitung dan menampilkan lintasan odometri vs ground truth.
- Mengevaluasi performa odometri melalui grafik dan perhitungan error.
- Menyediakan dokumentasi dan laporan untuk pembelajaran mahasiswa :contentReference[oaicite:2]{index=2}.

## ✨ Fitur
- Membaca parameter fisik robot langsung dari scene CoppeliaSim:
  - Jari-jari roda.
  - Jarak antar roda.
  - Time step simulasi.
- Menghitung lintasan odometri secara real-time.
- Membaca data ground truth (GT) dari CoppeliaSim.
- Proyeksi GT ke kerangka lokal awal robot untuk perbandingan.
- Menampilkan grafik:
  - $x(t)$ ODO vs GT
  - $y(t)$ ODO vs GT
  - yaw$(t)$ ODO vs GT
  - Trajectory (lintasan $x$–$y$)
  - Error $e_x(t)$, $e_y(t)$, $e_\theta(t)$
- Menghitung metrik ringkas:
  - RMSE posisi
  - Galat orientasi maksimum :contentReference[oaicite:3]{index=3}

## 🛠️ Software yang Dibutuhkan
- **Python 3**
- **CoppeliaSim Edu**
- **ZMQ Remote API** (sudah tersedia di dalam folder `programming/zmqRemoteApi/clients/python` pada instalasi CoppeliaSim)
- Library Python:
  - `coppeliasim-zmqremoteapi-client`
  - `matplotlib`


## ▶️ Tutorial Penggunaan
1. **Persiapan**
   - Instal Python 3.
   - Instal pustaka:
     ```bash
     pip install coppeliasim-zmqremoteapi-client matplotlib
     ```
   - Pastikan CoppeliaSim Edu dapat dijalankan.

2. **Penempatan File**
   - Simpan file scene CoppeliaSim, program Python, dan API Python ZMQ Remote dalam folder yang sama.
   - Pastikan path objek di scene sesuai dengan program:
     - `/PioneerP3DX`
     - `/rightMotor`
     - `/leftMotor`
     - `/rightMotor/rightWheel`

3. **Menjalankan Simulasi**
   - Buka CoppeliaSim → muat scene Pioneer P3DX.
   - Jalankan program Python:
     ```bash
     python odo_p3dx.py
     ```
   - Program otomatis akan:
     - Menghubungkan ke CoppeliaSim.
     - Memulai simulasi.
     - Membaca parameter roda.
     - Menghitung odometri dan membandingkan dengan ground truth.

4. **Menghentikan Simulasi**
   - Tekan tombol **Stop** di CoppeliaSim.
   - Program Python akan berhenti dan menampilkan hasil.

5. **Hasil yang Ditampilkan**
   - Jendela 1 (ODO vs GT):  
     - $x(t)$, $y(t)$, yaw$(t)$, dan lintasan $x$–$y$.
   - Jendela 2 (Error):  
     - $e_x(t)$, $e_y(t)$, $e_\theta(t)$.
   - Konsol:  
     - RMSE posisi dan galat orientasi maksimum :contentReference[oaicite:4]{index=4}.

## 📊 Kesimpulan
Program ini berhasil:
- Menghubungkan Python dengan CoppeliaSim melalui ZMQ Remote API.
- Menghitung odometri differential-drive secara real-time.
- Membaca ground truth dari simulasi dan membandingkannya dengan hasil odometri.
- Menampilkan visualisasi grafik serta metrik error kuantitatif.  

Secara keseluruhan, program telah berfungsi dengan baik untuk **analisis performa odometri pada robot Pioneer P3DX** dalam simulasi CoppeliaSim :contentReference[oaicite:5]{index=5}.

