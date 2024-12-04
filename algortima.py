INISIALISASI KONSTANTA
- Set ukuran minimal dan maksimal bola
- Set sudut belok (60 derajat)
- Set batas waktu operasi
- Set parameter PID dan filter

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
                > Maju
            LANJUT ke langkah berikutnya
            
        4. NAVIGASI UTAMA
            JIKA ada bola di tengah frame
                JIKA bola tersebut merah
                    SELAMA belum mencapai ukuran target
                        > Sesuaikan posisi ke bola merah
                        > Maju perlahan
                    
                    SETELAH mencapai bola merah
                        > Belok kiri 60 derajat
                        > Mulai pencarian bola hijau
                        
                    SAAT menemukan bola hijau
                        SELAMA belum mencapai ukuran target
                            > Sesuaikan posisi ke bola hijau
                            > Maju perlahan
                        
                        SETELAH mencapai bola hijau
                            > Belok kanan 60 derajat
            
            JIKA tidak ada bola terdeteksi
                JALANKAN POLA PENCARIAN
                    1. Coba pencarian spiral
                    2. Jika gagal, coba pencarian zig-zag
                    3. Jika masih gagal, kembali ke posisi awal
                    
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
