# KELOMPOK-3
input gambar

deteksi warna
ubah ke HSV
hijau = high intesity, low intesity
merah = high itensity, low intensity

cari kontur warna terpilih
cari area kontur terbesar
cari moment kontur terbesar (titik pusat)

bagi frame jadi 4 bagian

awal dari jalannya algoritma = bola hijau berada di kiri frame dan titik pusatnya di kordinat (sekian) dengan ukuran bola hijau (sekian) 
                              &&
                              bola merah berda di kanan frame dan titik pusatnya di kordinat (sekian) dengan ukuran bola merah (sekian)
                              = maju
if
ada bola ditengah frame
  if bola merah 
sesuaikan titik pusat bola merah dan maju sampai ukuran boal merah (sekian)
belok ke kiri sebesar 60 derajat
dan cari bola hijau, sesuaikan titik pusat bola hijau dan maju sampai ukuran bola hijau (sekian)
dan belok ke kanan sebesar 60 derajat
 if 
