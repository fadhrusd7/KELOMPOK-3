INISIALISASI KONSTANTA
- Set ukuran minimal dan maksimal bola
- Set sudut belok (60 derajat)
- Set batas waktu operasi
- set pembagian frame

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
            def belok
            pas = false
            JIKA titik pusat bola merah OR titik pusat bola hijau 
                = sesuai
            return pas

            SELAMA ada bola di tengah frame
                JIKA bola tersebut merah
                    SELAMA belum titik pusat bola belum sesuai dengan titik yang diinginkan 
                        #Sesuaikan posisi ke bola merah 
                        JIKA titik pusat bola merah berada di bagian kanan frame 
                        > belok kanan sampai titik (yang diatur) sesuai dengan titik pusat bola dengan batas sampai ( 0 <= 60 derajat )
                        > titik pusat bola dan titik (yang inginkan (tegah frame)) sesuai = True
                        Jika titik pusat bola merah berada di bagian kanan frame
                        > bolek kiri sampai titik (yang diatur) sesuai dengan titik pusat bola dengan batas sampai ( 0 <= 60 derajat )
                        > titik pusat bola dan titik (yang inginkan (tegah frame)) sesuai = True
                        return belok
                    
                 JIKA bola tersebut hijau
                     SELAMA belum titik pusat bola belum sesuai dengan titik yang diinginkan 
                        #sesuaikan posisi ke bola hijau
                          JIKA titik pusat bola merah berada di bagian kanan frame 
                        > belok kanan sampai titik (yang diatur) sesuai dengan titik pusat bola dengan batas sampai ( 0 <= 60 derajat )
                        > titik pusat bola dan titik (yang inginkan (tegah frame)) sesuai = True
                          JIKA titik pusat bola merah berada di bagian kanan frame
                        > bolek kiri sampai titik (yang diatur) sesuai dengan titik pusat bola dengan batas sampai ( 0 <= 60 derajat )
                        > titik pusat bola dan titik (yang inginkan (tegah frame)) sesuai = True
                        return belok

                JIKA bola merah ATAU hijau terdeteksi
                > maju 1 meter dan berhenti

                def jalan
                sesuai = false
                SELAMA ukuran bola belum sesuai 
                > maju
                JIKA ukuran bola sesuai 
                return sesuai
    
                    
        5. PENANGANAN ERROR
            JIKA terjadi timeout
                > Hentikan operasi
                > Reset sistem
                > Mulai ulang navigasi
                
            JIKA terjadi error
                > Catat di log
                
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
