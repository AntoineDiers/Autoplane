import { Component, ChangeDetectorRef, AfterViewInit } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { CommonModule } from '@angular/common';
import { ModeState } from './Structs/ModeState';
import { LeafletModule } from '@bluehalo/ngx-leaflet';
import * as Leaflet from 'leaflet';

import { Mode } from './Structs/Mode';

import { listen } from '@tauri-apps/api/event';

@Component({
    selector: 'app-root',
    standalone: true,
    imports: [RouterOutlet, CommonModule, LeafletModule],
    templateUrl: './app.component.html',
    styleUrl: './app.component.css'
    })
export class AppComponent implements AfterViewInit {

    // Structs
    Mode = Mode;
    ModeState = ModeState;

    // Autopilot state
    mode : Mode = Mode.IDLE;

    // Pose
    roll_deg :      number = 0;
    pitch_deg :     number = 0;
    heading_deg :   number = 0;
    latitude_deg :  number = 0;
    longitude_deg : number = 0;
    altitude_m :    number = 0;

    ping_ms : number | null = null;

    map : Leaflet.Map | null = null;
    marker : Leaflet.Marker | null = null;

    constructor(private changeDetector: ChangeDetectorRef) 
    {
        listen<number|null>('ping', (event) => { this.ping_ms = event.payload; this.changeDetector.detectChanges(); });

        listen<number>('roll',      (event) => { this.roll_deg = event.payload;         this.changeDetector.detectChanges(); });
        listen<number>('pitch',     (event) => { this.pitch_deg = event.payload;        this.changeDetector.detectChanges(); });
        listen<number>('heading',   (event) => { this.heading_deg = event.payload;      this.changeDetector.detectChanges(); });
        listen<number>('latitude',  (event) => { this.latitude_deg = event.payload;     this.changeDetector.detectChanges(); });
        listen<number>('longitude', (event) => { this.longitude_deg = event.payload;    this.changeDetector.detectChanges(); });
        listen<number>('altitude',  (event) => { this.altitude_m = event.payload;       this.changeDetector.detectChanges(); });
        
        // Update plane position on map
        setInterval(() => { this.updateAutoplaneMarker(); }, 100);
    }

    ngAfterViewInit() 
    {
        this.map = Leaflet.map('map', 
        {
            layers: 
            [
                new Leaflet.TileLayer(
                    'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', 
                    {attribution: '&copy; OpenStreetMap contributors'} as Leaflet.TileLayerOptions)
            ],
            zoom: 12,
            center: new Leaflet.LatLng(49.44, 1.09),
            zoomControl: false,
        });

        this.marker = Leaflet.marker([0,0], 
        {
            icon: Leaflet.divIcon({
                className: 'leaflet-marker',
                iconSize: [50, 50],
                iconAnchor: [25, 25],
                html: `<img id="leaflet-marker-image" src="/assets/icons/attitude/top.png" width="50" height="50">`
            })
        }).addTo(this.map);
    }

    updateAutoplaneMarker()
    {
        if(!this.map || !this.marker) { return; }

        this.marker.setLatLng([this.latitude_deg, this.longitude_deg]);
        const img = document.getElementById('leaflet-marker-image') as HTMLImageElement;
        if(img)
        {
            img.style.transform = `rotate(${this.heading_deg}deg)`;
        }
    }

    getSpeedKmh() : number {
        
        return 0;
        //return this.state.navigation_data.speed_m_s * 3.6;
    }

    getModeState(mode : Mode)
    {
        if (this.mode === mode) {
            return ModeState.SELECTED;
        } 
        else 
        {
            switch (this.mode) 
            {
                case Mode.IDLE: if (mode === Mode.TAKEOFF) return ModeState.AVAILABLE; break;
                case Mode.TAKEOFF: if (mode === Mode.MANUAL) return ModeState.AVAILABLE; break;
                case Mode.MANUAL: if (mode === Mode.AUTO || mode === Mode.LANDING) return ModeState.AVAILABLE; break;
                case Mode.AUTO: if (mode === Mode.MANUAL) return ModeState.AVAILABLE; break;
                case Mode.LANDING: if (mode === Mode.IDLE) return ModeState.AVAILABLE; break;
            }
        }

        return ModeState.UNAVALABLE;
    }

    onSetMode(mode : Mode)
    {
        if (this.getModeState(mode) === ModeState.AVAILABLE) {
            this.mode = mode;
        }
    }
}
