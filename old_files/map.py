import folium
tileset = 'http://localhost:8079/{z}/{x}/{y}.jpeg'
m = folium.Map(tiles=tileset,zoom_start=0,max_zoom=3,min_zoom=1, attr='Test',no_wrap=True)
m.save('map.html')