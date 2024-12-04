INISIALISASI KONSTANTA
- Set ukuran minimal dan maksimal bola
- Set sudut belok (60 derajat)
- Set batas waktu operasi

PROGRAM UTAMA
    MULAI LOOP UTAMA
        1. AMBIL DAN PROSES GAMBAR
            - Ambil frame dari kamera
            - Kurangi noise gambar
            - Ubah ke format HSV
            
        2. DETEKSI BOLA
            - Cari bola hijau
                > Filter warna hijau
                > Cari kontur terbesar
                > Hitung posisi dan ukuran
                > Validasi hasil deteksi
            
            - Cari bola merah
                > Filter warna merah
                > Cari kontur terbesar
                > Hitung posisi dan ukuran
                > Validasi hasil deteksi
                
        3. CEK POSISI AWAL
            JIKA bola hijau di kiri DAN bola merah di kanan
            sesuaikan bola merah ada di frame (yang ditentukan) dan bola hijau berada di frame (yang telah ditentukan)
                > Maju
            LANJUT ke langkah berikutnya
            
        4. NAVIGASI UTAMA
            SELAMA ada bola di tengah frame
                JIKA bola tersebut merah
                    SELAMA belum mencapai ukuran target
                        > Sesuaikan posisi ke bola merah
                        > Maju perlahan
                    
                    SETELAH mencapai bola merah dengan ukuran yang diinginkan 
                        > Belok kiri 60 derajat
    
                 JIKA bola tersebut hijau
                     SELAMA belum mencapai ukuran target
                        > sesuaikan posisi ke bola hijau
                        > muju perlahan
                     Setelah mencapai bola hijau dengan ukuran yang dinginkan
                        > Belok ke kanan 60 derajat

                 JIKA tidak terdeteksi bola hijau dan merah 
                        > maju 1 meter 
                        > berhenti

    
                    
        5. PENANGANAN ERROR
            JIKA terjadi timeout
                > Hentikan operasi
                > Reset sistem
                > Mulai ulang navigasi
                
            JIKA terjadi error
                > Catat di log
                > Jalankan prosedur darurat
                
        6. LOGGING
            - Catat status operasi
            - Catat error yang terjadi
            - Catat posisi dan ukuran bola
            
    ULANGI LOOP UTAMA

FUNGSI VALIDASI DETEKSI
    CEK:
    - Ukuran dalam rentang yang valid
    - Bentuk mendekati lingkaran
    - Intensitas warna sesuai
    RETURN hasil validasi

FUNGSI PENYESUAIAN POSISI
    - Hitung error posisi
    - Hitung adjustment PID
    - Terapkan ke motor
