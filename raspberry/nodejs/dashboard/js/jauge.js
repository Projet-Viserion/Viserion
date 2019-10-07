let jauge_bat = new JustGage({
                id: "battery",
                value: 90,
                min: 0,
                max: 100,
                title: "Battery",
                levelColors: [
                    "#FF0000",
                    "#FFA500",
                    "#32CD32"
                ]
            });

            let jauge_r = new JustGage({
                id: "rcurrent",
                value: 1000,
                min: 0,
                max: 1100,
                title: "Moteur Droit",
            });

            let jauge_l = new JustGage({
                id: "lcurrent",
                value: 1000,
                min: 0,
                max: 1100,
                title: "Moteur Gauche",
            });

            let jauge_f = new JustGage({
                id: "fcurrent",
                value: 300,
                min: 0,
                max: 400,
                title: "Moteur Volant",
            });

            let jauge_volant = new JustGage({
                id: "volant",
                value: 0,
                min: -45,
                max: 45,
                title: "Direction",
                levelColors: [
                    "#7B68EE"
                ]
            });

            let jauge_speedD = new JustGage({
                id: "speedD",
                value: 0,
                min: 0,
                max: 100,
                title: "Vitesse Droite",
            });

            let jauge_speedG = new JustGage({
                id: "speedG",
                value: 0,
                min: 0,
                max: 100,
                title: "Vitesse Gauche",
            });
