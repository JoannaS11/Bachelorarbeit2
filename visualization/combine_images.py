import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Bilder laden (ersetze durch den Pfad zu deinen Bildern)
img1 = mpimg.imread('/home/yn86eniw/111_images_BA/main_part/intestine_265000_centra_pcd.png')
img2 = mpimg.imread('/home/yn86eniw/111_images_BA/main_part/colon_subtr_centr_pcd.png')
img3 = mpimg.imread('/home/yn86eniw/111_images_BA/main_part/anim_haustren_centra_pcd.png')
#img4 = mpimg.imread('bild4.png')

# Erstellen einer 2x2-Grid für Subplots
fig, axs = plt.subplots(2, 2, figsize=(10, 10))

# Anzeige der Bilder in den Subplots
axs[0, 0].imshow(img3)
axs[0, 0].set_title('Bild 1')
axs[0, 1].imshow(img2)
axs[0, 1].set_title('Bild 2')
axs[1, 0].imshow(img1)
axs[1, 0].set_title('Bild 3')
#axs[1, 1].imshow(img4)
#axs[1, 1].set_title('Bild 4')

# Achsen entfernen, wenn nicht benötigt
for ax in axs.flat:
    ax.axis('off')

# Layout anpassen und Fenster anzeigen
plt.tight_layout()
plt.show()