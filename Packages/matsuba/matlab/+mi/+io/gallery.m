function t = gallery()
%MI.IO.GALLERY List available Mitsuba 3 gallery scenes.
%   T = MI.IO.GALLERY() returns a table of downloadable scenes from the
%   Mitsuba 3 gallery (https://mitsuba.readthedocs.io/en/stable/src/gallery.html).
%
%   The table has columns:
%     Name     - Short identifier used with mi.io.downloadScene (string)
%     Title    - Human-readable scene name (string)
%     Category - Scene category: "simple", "object", "architecture", "banner" (string)
%
%   Example:
%       t = mi.io.gallery();
%       disp(t);
%       scene = mi.io.downloadScene("kitchen");
%
%   See also mi.io.downloadScene

    Name = [
        % Simple scenes
        "cornell-box"; "matpreview"; "veach-bidir"; "veach-mis"; "veach-ajar"; "volumetric-caustic"
        % Single object
        "car"; "car2"; "coffee"; "dragon"; "spaceship"; "lamp"; "teapot"
        "teapot-full"; "lego"; "rover"; "hair-curl"; "curly-hair"; "straight-hair"; "furball"
        % Architecture
        "bathroom"; "bathroom2"; "bedroom"; "classroom"; "dining-room"; "kitchen"
        "living-room"; "living-room-2"; "living-room-3"; "staircase"; "staircase2"
        "glass-of-water"; "house"
        % Banners
        "banner_01"; "banner_02"; "banner_03"; "banner_04"; "banner_05"; "banner_06"; "banner_07"
    ];

    Title = [
        "Cornell Box"; "Material preview"; "Veach bidir"; "Veach MIS"; "Veach Ajar"; "Volumetric caustics"
        "Pontiac GTO 67"; "Old vintage car"; "Coffee Maker"; "Dragon"; "Spaceship"; "Lamp"; "Teapot"
        "Teapot full"; "Lego Bulldozer"; "Sci-Fi Rover"; "Hair curls"; "Curly hair"; "Straight hair"; "Fur ball"
        "Bathroom"; "Salle de bain"; "Bedroom"; "Japanese Classroom"; "The Breakfast Room"; "Country Kitchen"
        "Grey & White Room"; "The White Room"; "Modern Living Room"; "Wooden Staircase"; "Modern Hall"
        "Glass of water"; "Victorian Style House"
        "Banner 1"; "Banner 2"; "Banner 3"; "Banner 4"; "Banner 5"; "Banner 6"; "Banner 7"
    ];

    Category = [
        repmat("simple", 6, 1)
        repmat("object", 14, 1)
        repmat("architecture", 13, 1)
        repmat("banner", 7, 1)
    ];

    t = table(Name, Title, Category);
end
