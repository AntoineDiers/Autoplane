import { Component } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { CommonModule } from '@angular/common';
import { State } from './Structs/State';
import { ConnectionState } from './Structs/ConnectionState';
import { Mode } from './Structs/Mode';
import { ModeState } from './Structs/ModeState';
import { LeafletModule } from '@asymmetrik/ngx-leaflet';
import * as Leaflet from 'leaflet';

@Component({
    selector: 'app-root',
    standalone: true,
    imports: [RouterOutlet, CommonModule, LeafletModule],
    templateUrl: './app.component.html',
    styleUrl: './app.component.css'
    })
export class AppComponent {

    Mode = Mode;
    ModeState = ModeState;

    state : State = {
        navigation_data: {
            attitude: {
                roll_deg: 0,
                pitch_deg: 0,
                yaw_deg: 0
            },
            position: {
                latitude_deg: 0,
                longitude_deg: 0,
                altitude_m: 0
            },
            speed_m_s: 0,
        },
        mode: Mode.IDLE
    };

    connection_state : ConnectionState = { connected: true, ping_ms: 50 };

    map : Leaflet.Map | null = null; 
    marker : Leaflet.Marker | null = null;

    constructor(private http: HttpClient) 
    {
        setInterval(() => {
            this.http.get<State>('http://' + window.location.hostname + ':8080/state').subscribe(data => { this.state = data; });
            if(!this.map) 
            {
                this.map = Leaflet.map('map', {
                layers: [
                    new Leaflet.TileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                        attribution: '&copy; OpenStreetMap contributors'
                    } as Leaflet.TileLayerOptions)
                ],
                zoom: 12,
                center: new Leaflet.LatLng(49.44, 1.09),
                zoomControl: false,
            });
                  }
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


        setInterval(() => { this.updateGamepadInputs(); }, 100);
    }

    getSpeedKmh() : number {
        return this.state.navigation_data.speed_m_s * 3.6;
    }

    updateGamepadInputs()
    {
        const gamepad = navigator.getGamepads()[0];
        if(!gamepad) { return; }
        
        this.http.post('http://' + window.location.hostname + ':8080/rc_inputs', {
            throttle: (gamepad.buttons[7].value),
            brake: (gamepad.buttons[6].value),
            elevation: (gamepad.axes[1]),
            steering: (gamepad.axes[2])
        }).subscribe();
    }

    getModeState(mode : Mode)
    {
        if (this.state.mode === mode) {
            return ModeState.SELECTED;
        } 
        else 
        {
            switch (this.state.mode) 
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
            this.state.mode = mode;
        }
    }
}
