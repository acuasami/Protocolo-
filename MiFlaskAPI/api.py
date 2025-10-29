from flask import Flask, jsonify, request
from flask_cors import CORS
import psycopg2
import os
import logging

# Configurar logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)  # Permite solicitudes desde Android

# --- Configuración Railway ---
DB_CONFIG = {
    'host': 'tramway.proxy.rlwy.net',
    'port': 31631,
    'database': 'railway',
    'user': 'postgres',
    'password': 'KAGJhRklTcsevGqKEgCNPfmdDiGzsLyQ'
}

def get_db_connection():
    """Conecta a PostgreSQL en Railway"""
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        return conn
    except Exception as e:
        logger.error(f"❌ Error conectando a PostgreSQL: {e}")
        raise e

# --- ENDPOINT PRINCIPAL ---
@app.route("/")
def home():
    return jsonify({
        "status": "active", 
        "message": "🚀 API Flask - ONGs México",
        "version": "2.0",
        "endpoints": {
            "health": "/api/health",
            "login": "/api/auth/login",
            "register": "/api/auth/register", 
            "ongs": "/api/ongs",
            "ongs_by_state": "/api/ongs/<estado>",
            "debug": "/api/debug/db"
        }
    })

# --- ENDPOINT DE SALUD ---
@app.route("/api/health", methods=['GET'])
def health_check():
    """Verifica que la API y base de datos estén funcionando"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute('SELECT version()')
        db_version = cur.fetchone()
        cur.close()
        conn.close()
        
        return jsonify({
            "status": "healthy",
            "message": "✅ API y base de datos conectadas",
            "database": "PostgreSQL en Railway",
            "database_version": db_version[0] if db_version else "unknown"
        })
    except Exception as e:
        return jsonify({
            "status": "unhealthy", 
            "message": f"❌ Error: {str(e)}"
        }), 500

# --- ENDPOINTS DE AUTENTICACIÓN ---
@app.route('/api/auth/login', methods=['POST'])
def login():
    """Login de usuario"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({'success': False, 'message': 'Datos no proporcionados'}), 400
        
        username = data.get('username', '').strip()
        password = data.get('password', '').strip()

        if not username or not password:
            return jsonify({'success': False, 'message': 'Usuario y contraseña requeridos'}), 400

        conn = get_db_connection()
        cur = conn.cursor()
        
        # Verificar credenciales - intentar con diferentes nombres de tabla
        user = None
        table_names = ['usuario', 'users', 'user']  # Posibles nombres de tabla
        
        for table in table_names:
            try:
                cur.execute(f"SELECT id, nombre FROM {table} WHERE nombre = %s AND contraseña = %s", (username, password))
                user = cur.fetchone()
                if user:
                    break
            except:
                continue
        
        cur.close()
        conn.close()

        if user:
            logger.info(f"✅ Login exitoso para: {username}")
            return jsonify({
                'success': True,
                'message': 'Login exitoso',
                'user': {
                    'id': user[0],
                    'nombre': user[1]
                }
            })
        else:
            logger.warning(f"❌ Login fallido para: {username}")
            return jsonify({
                'success': False,
                'message': 'Credenciales incorrectas'
            }), 401

    except Exception as e:
        logger.error(f"💥 Error en login: {e}")
        return jsonify({
            'success': False,
            'message': 'Error del servidor'
        }), 500

@app.route('/api/auth/register', methods=['POST'])
def register():
    """Registro de nuevo usuario"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({'success': False, 'message': 'Datos no proporcionados'}), 400
        
        username = data.get('username', '').strip()
        password = data.get('password', '').strip()

        if not username or not password:
            return jsonify({'success': False, 'message': 'Usuario y contraseña requeridos'}), 400

        if len(password) < 4:
            return jsonify({'success': False, 'message': 'La contraseña debe tener al menos 4 caracteres'}), 400

        conn = get_db_connection()
        cur = conn.cursor()
        
        # Verificar si el usuario ya existe - intentar con diferentes nombres de tabla
        user_exists = False
        table_names = ['usuario', 'users', 'user']
        
        for table in table_names:
            try:
                cur.execute(f"SELECT id FROM {table} WHERE nombre = %s", (username,))
                if cur.fetchone():
                    user_exists = True
                    break
            except:
                continue

        if user_exists:
            cur.close()
            conn.close()
            return jsonify({
                'success': False,
                'message': 'El usuario ya existe'
            }), 400

        # Crear nuevo usuario - usar la primera tabla que funcione
        user_id = None
        for table in table_names:
            try:
                cur.execute(f"INSERT INTO {table} (nombre, contraseña) VALUES (%s, %s) RETURNING id", (username, password))
                user_id = cur.fetchone()[0]
                conn.commit()
                break
            except:
                conn.rollback()
                continue
        
        cur.close()
        conn.close()

        if user_id:
            logger.info(f"✅ Usuario registrado: {username} (ID: {user_id})")
            return jsonify({
                'success': True,
                'message': 'Usuario registrado exitosamente',
                'user_id': user_id
            })
        else:
            return jsonify({
                'success': False,
                'message': 'No se pudo crear el usuario - tabla no encontrada'
            }), 500

    except Exception as e:
        logger.error(f"💥 Error en registro: {e}")
        return jsonify({
            'success': False,
            'message': 'Error del servidor'
        }), 500

# --- ENDPOINTS DE ONGs ---
@app.route("/api/ongs", methods=['GET'])
def get_ongs():
    """Obtener todas las ONGs"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        # Intentar diferentes nombres de tabla
        table_names = ['ongs', 'ong', 'organizaciones']
        ongs_data = []
        
        for table in table_names:
            try:
                cur.execute(f"""
                    SELECT nombre, latitud, longitud, descripcion, telefono, estado, municipio 
                    FROM {table} 
                    WHERE latitud IS NOT NULL AND longitud IS NOT NULL
                    LIMIT 50
                """)
                ongs_data = cur.fetchall()
                if ongs_data:
                    logger.info(f"✅ Tabla encontrada: {table}")
                    break
            except Exception as e:
                logger.warning(f"Tabla {table} no encontrada: {e}")
                continue

        cur.close()
        conn.close()

        # Convertir a JSON
        ongs_list = []
        for ong in ongs_data:
            ongs_list.append({
                'nombre': ong[0] or 'Sin nombre',
                'latitud': float(ong[1]) if ong[1] else 0.0,
                'longitud': float(ong[2]) if ong[2] else 0.0,
                'descripcion': ong[3] or 'Sin descripción',
                'telefono': ong[4] or 'Sin teléfono',
                'estado': ong[5] or 'Sin estado',
                'municipio': ong[6] or 'Sin municipio'
            })

        logger.info(f"📊 Se enviaron {len(ongs_list)} ONGs")
        return jsonify({
            'success': True,
            'ongs': ongs_list,
            'count': len(ongs_list)
        })

    except Exception as e:
        logger.error(f"💥 Error obteniendo ONGs: {e}")
        return jsonify({
            'success': False,
            'message': f'Error obteniendo ONGs: {str(e)}'
        }), 500

@app.route('/api/ongs/<estado>', methods=['GET'])
def get_ongs_by_state(estado):
    """Obtener ONGs por estado"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        # Intentar diferentes nombres de tabla
        table_names = ['ongs', 'ong', 'organizaciones']
        ongs_data = []
        
        for table in table_names:
            try:
                cur.execute(f"""
                    SELECT nombre, latitud, longitud, descripcion, telefono, estado, municipio 
                    FROM {table} 
                    WHERE estado ILIKE %s AND latitud IS NOT NULL AND longitud IS NOT NULL
                """, (f'%{estado}%',))
                ongs_data = cur.fetchall()
                if ongs_data:
                    break
            except:
                continue

        cur.close()
        conn.close()

        ongs_list = []
        for ong in ongs_data:
            ongs_list.append({
                'nombre': ong[0] or 'Sin nombre',
                'latitud': float(ong[1]) if ong[1] else 0.0,
                'longitud': float(ong[2]) if ong[2] else 0.0,
                'descripcion': ong[3] or 'Sin descripción',
                'telefono': ong[4] or 'Sin teléfono',
                'estado': ong[5] or 'Sin estado',
                'municipio': ong[6] or 'Sin municipio'
            })

        return jsonify({
            'success': True,
            'ongs': ongs_list,
            'count': len(ongs_list)
        })

    except Exception as e:
        logger.error(f"💥 Error obteniendo ONGs por estado: {e}")
        return jsonify({
            'success': False,
            'message': 'Error obteniendo ONGs'
        }), 500

# --- ENDPOINT DE DIAGNÓSTICO ---
@app.route("/api/debug/db", methods=['GET'])
def debug_db():
    """Diagnóstico de la base de datos"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        # Ver tablas existentes
        cur.execute("""
            SELECT table_name 
            FROM information_schema.tables 
            WHERE table_schema = 'public'
            ORDER BY table_name
        """)
        tablas = [row[0] for row in cur.fetchall()]
        
        # Contar registros en cada tabla
        counts = {}
        for tabla in tablas:
            try:
                cur.execute(f"SELECT COUNT(*) FROM {tabla}")
                counts[tabla] = cur.fetchone()[0]
            except:
                counts[tabla] = "error"
        
        cur.close()
        conn.close()
        
        return jsonify({
            "status": "success",
            "tablas": tablas,
            "conteos": counts,
            "total_tablas": len(tablas)
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# --- ENDPOINT ORIGINAL (para compatibilidad) ---
@app.route("/ongs", methods=['GET'])
def get_ongs_original():
    """Endpoint original - para compatibilidad"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute("SELECT * FROM ong LIMIT 5;")
        rows = cur.fetchall()
        columns = [desc[0] for desc in cur.description]
        data = [dict(zip(columns, row)) for row in rows]
        cur.close()
        conn.close()
        return jsonify(data)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    port = int(os.environ.get('PORT', 5000))
    app.run(host='0.0.0.0', port=port, debug=False)

