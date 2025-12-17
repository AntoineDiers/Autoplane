import { Component, ChangeDetectorRef, AfterViewInit } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { CommonModule } from '@angular/common';
import { ConnectionState } from './Structs/ConnectionState';
import { ModeState } from './Structs/ModeState';
import { LeafletModule } from '@bluehalo/ngx-leaflet';
import * as Leaflet from 'leaflet';

import { Position, DefaultPosition } from './Structs/Position'
import { Attitude, DefaultAttitude } from './Structs/Attitude';
import { Mode, DefaultMode } from './Structs/Mode';

import { listen } from '@tauri-apps/api/event';

@Component({
    selector: 'app-root',
    standalone: true,
    imports: [RouterOutlet, CommonModule, LeafletModule],
    templateUrl: './app.component.html',
    styleUrl: './app.component.css'
    })
export class AppComponent implements AfterViewInit {

    Mode = Mode;
    ModeState = ModeState;

    mode :      Mode = DefaultMode;
    attitude :  Attitude = DefaultAttitude;
    position :  Position = DefaultPosition;

    connection_state : ConnectionState = { connected: true, ping_ms: 50 };

    map : Leaflet.Map | null = null;
    marker : Leaflet.Marker | null = null;

    constructor(private changeDetector: ChangeDetectorRef) 
    {
        listen<Attitude>('attitude', (event) => { this.attitude = event.payload; this.changeDetector.detectChanges(); });

        /*
        setInterval(() => {
            this.http.get<State>('http://' + window.location.hostname + ':8080/state').subscribe(data => { this.state = data; });
            if(!this.marker)
            {
                this.marker = Leaflet.marker([0,0], 
                {
                    icon: Leaflet.divIcon({
                        className: 'leaflet-marker',
                        iconSize: [100, 100],
                        iconAnchor: [50, 50],
                        html: `<img id="leaflet-marker-image" src="icons/attitude/top.png" width="30" height="30">`
                    })
                }).addTo(this.map);
            }
            this.marker.setLatLng([this.state.navigation_data.position.latitude_deg, this.state.navigation_data.position.longitude_deg]);
            this.map.setView([this.state.navigation_data.position.latitude_deg, this.state.navigation_data.position.longitude_deg]);
            const img = document.getElementById('leaflet-marker-image') as HTMLImageElement;
            if(img)
            {
                img.style.transform = `rotate(${this.state.navigation_data.attitude.yaw_deg}deg)`;
            }
        }, 100);

        setInterval(() => {
            this.connection_state.ping_ms = this.connection_state.connected ? 50 + 20 * (Math.random() - 0.5) : null;
        }, 1000);

        setInterval(() => {
            this.connection_state.connected = Math.random() > 0.1 ? true : false;
        }, 5000);


        setInterval(() => { this.updateGamepadInputs(); }, 100);*/
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
